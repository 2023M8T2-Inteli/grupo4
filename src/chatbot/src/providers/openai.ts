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

export async function getPointOpenAI(message: Message, client: Client, points) {
  let prompt = process.env.PROMPT_OPENAI_POINTS;

  let question = `Lista de pontos: ${JSON.stringify(
    points
  )}. Identifique a responsta do usuário com base na lista de pontos e depois coloque as coordenadas do ponto em formato de float. Pergunta do usuário: ${
    message.body
  }`;

  try {
    const response = await openai.createChatCompletion({
      model: 'gpt-4',
      messages: [
        { role: 'system', content: prompt },
        { role: 'user', content: question },
      ],
    });

    let pointResponse = response.data.choices[0].message?.content;

    const regex: RegExp =
      /-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?/gi;
    const match = pointResponse?.match(regex);
    const socket = io('http://10.128.68.115:3000');
    if (match) {
      match.forEach((coordinateString) => {
        // Splitting the matched string into individual numbers
        const parts = coordinateString
          .split(',')
          .map((part) => parseFloat(part.trim()));
        const [x, y, z] = parts;
        socket.emit('enqueue', { x, y, z });
        message.reply(pointResponse as any);
      });
    } else {
      message.reply('Não consegui encontrar o ponto. Tente novamente.');
    }
  } catch (e) {
    terminal.printError(e);
    client.sendMessage(message.from, stringify(e));
  }
}

export async function transcribeOpenAI(
  Message: Message,
  Client: Client,
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

export async function speechOpenAI(
  message: Message,
  client: Client,
  text: string
): Promise<String> {
  const url = process.env.TTS_URL || 'https://api.openai.com/v1/audio/speech';
  let response;
  try {
    response = await axios.post(
      url,
      {
        model: 'tts-1',
        voice: process.env.TTS_VOICE,
        input: text,
      },
      {
        headers: {
          Authorization: `Bearer ${process.env.OPENAI_API_KEY}`,
          'Content-Type': 'application/json',
        },
        responseType: 'arraybuffer',
      }
    );
  } catch (e) {
    console.error(e);
    return '';
  } finally {
    const audioBase64 = Buffer.from(response.data, 'binary').toString('base64');
    const messageMedia = new MessageMedia('audio/ogg', audioBase64);
    await client.sendMessage(message.from, messageMedia, {
      sendAudioAsVoice: true,
    });
    return audioBase64;
  }
}
