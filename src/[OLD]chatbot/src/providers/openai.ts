import fs from 'fs';
import axios from 'axios';
import path from 'path';
import { randomUUID } from 'crypto';
import FormData from 'form-data';
import { Configuration, OpenAIApi } from 'openai';
import ffmpeg from 'fluent-ffmpeg';
import * as terminal from '../cli/ui';
import { Client, Message, MessageMedia } from 'whatsapp-web.js';
import { io } from 'socket.io-client';
import { stringify } from 'querystring';
import { PrismaClient } from '@prisma/client';
import UserService from '../models/user';
import ToolService from '../models/tool';
import PointService from '../models/point';
import OrderService from '../models/order';
import { speechGoogle } from './google';
const prisma = new PrismaClient();
const userService = new UserService(prisma);
const toolService = new ToolService(prisma);
const pointService = new PointService(prisma);
const orderService = new OrderService(prisma);

export let openai: OpenAIApi;

export function initOpenAI() {
  const apiKey = process.env.OPENAI_API_KEY;
  if (!apiKey) {
    throw new Error(
      'A chave de API do OpenAI não está definida nas variáveis de ambiente.'
    );
  }

  openai = new OpenAIApi(
    new Configuration({
      apiKey: apiKey,
    })
  );
}

export async function getPointOpenAI(message: Message, client: Client) {
  const points = await JSON.stringify(await pointService.getPoints());
  let prompt = process.env.PROMPT_OPENAI_POINTS;
  let question = `Lista de pontos: ${points}. \n Identifique a responsta do usuário com base na lista de pontos e depois coloque as coordenadas do ponto em formato de float e responda apenas em português do Brasil. Pergunta do usuário: ${message.body}`;

  try {
    const response = await openai.createChatCompletion({
      model: 'gpt-4',
      messages: [
        { role: 'system', content: prompt },
        { role: 'user', content: question },
      ],
    });
    let responseText = response.data.choices[0].message?.content;
    await extractPoints(message, client, responseText);
  } catch (e) {
    terminal.printError(e);
    client.sendMessage(message.from, stringify(e));
  }
}

async function extractPoints(
  message: Message,
  client: Client,
  pointResponse: any
) {
  const regex: RegExp =
    /-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?/gi;
  const match = pointResponse?.match(regex);
  const socket = io(process.env.SOCKET_URL || '');

  if (match) {
    match.forEach(async (coordinateString) => {
      // Splitting the matched string into individual numbers
      const parts = coordinateString
        .split(',')
        .map((part) => parseFloat(part.trim()));
      const [x, y, z] = parts;
      socket.emit('enqueue', { x, y, z });
      const point = await pointService.getPoint(x, y, z);
      if (point) await orderService.updateOrder(message.from, point.id);
      await speechGoogle(message, client, pointResponse);
      await userService.updateRequestUser(message.from, 1);
      client.sendMessage(message.from, 'Pedido finalizado com sucesso!');
    });
  } else {
    message.reply('Não consegui encontrar o ponto. Tente novamente.');
  }
}

async function extractToolId(message: Message, client: Client, response: any) {
  const regex: RegExp =
    /[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}/g;
  const match = response?.match(regex);
  if (match) {
    match.forEach(async (idString) => {
      // Splitting the matched string into individual numbers
      orderService.createOrder(message.from, idString);
      await speechGoogle(message, client, response);
    });
  } else {
    message.reply('Não consegui encontrar o ponto. Tente novamente.');
  }
}

export async function getToolOpenAI(message: Message, client: Client) {
  let prompt = process.env.PROMPT_OPENAI_TOOLS;
  const tools = JSON.stringify(await toolService.getTools());
  let question = `Lista de peça: ${tools}. Com base na mensagem abaixo, identifique a melhor peça a ser usando nesse cenário ou solicitação feita com base na lista e em caso de não possuir na lista me indique uma outra que eu posso utilizar: ${message.body}`;

  try {
    const response = await openai.createChatCompletion({
      model: 'gpt-4',
      messages: [
        { role: 'system', content: prompt },
        { role: 'user', content: question },
      ],
    });

    let responseChat = response.data.choices[0].message?.content;
    await extractToolId(message, client, responseChat);
    client.sendMessage(message.from, 'Certo, você pode me dizer onde está?');
    message.reply('Você pode me dizer onde você está?');
    const points = await pointService.getPoints();
    let listPoints = '';
    if (points && points.length > 0) {
      for (const point of points) {
        listPoints += '*' + point.name + '*\n';
      }
      message.reply(listPoints);
    }
  } catch (e) {
    terminal.printError(e);
    client.sendMessage(message.from, stringify(e));
  }
}

export async function transcribeOpenAI(
  audioBuffer: Buffer
): Promise<{ text: string; language: string }> {
  const url =
    process.env.TRANSCRIPTION_URL ||
    'https://api.openai.com/v1/audio/transcriptions';
  let language = process.env.TRANSCRIPTION_LANGUAGE || 'pt-BR';
  const oggPath = path.join(__dirname + '/media/', randomUUID() + '.ogg');
  const wavFilename = randomUUID() + '.wav';
  const wavPath = path.join(__dirname + '/media/', wavFilename);
  fs.writeFileSync(oggPath, audioBuffer);
  try {
    await convertOggToWav(oggPath, wavPath);
  } catch (e) {
    fs.unlinkSync(oggPath);
    return {
      text: '',
      language,
    };
  }

  const formData = new FormData();

  formData.append('file', audioBuffer, {
    filename: wavFilename,
    contentType: 'audio/wav',
  });
  formData.append('model', 'whisper-1');
  formData.append('response_format', 'json');
  let response;
  try {
    // response = await fetch(url, options);
    response = await axios.post(url, formData, {
      headers: {
        ...formData.getHeaders(),
        Authorization: `Bearer ${process.env.OPENAI_API_KEY}`,
      },
    });
  } catch (e) {
    console.error(e);
    return {
      text: '',
      language: language,
    };
  } finally {
    fs.unlinkSync(oggPath);
    fs.unlinkSync(wavPath);
    const transcription = await response.data;

    return {
      text: transcription.text,
      language,
    };
  }
}

async function convertOggToWav(
  oggPath: string,
  wavPath: string
): Promise<void> {
  return new Promise((resolve, reject) => {
    ffmpeg(oggPath)
      .toFormat('wav')
      .outputOptions('-acodec pcm_s16le')
      .output(wavPath)
      .on('end', () => resolve())
      .on('error', (err) => reject(err))
      .run();
  });
}