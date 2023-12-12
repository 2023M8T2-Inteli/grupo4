import { Injectable } from '@nestjs/common';
import { randomUUID } from 'crypto';
import { writeFileSync } from 'fs';
import { Configuration, OpenAIApi } from 'openai';
import path from 'path';
import FormData from 'form-data';
import { Client, Message, MessageMedia } from 'whatsapp-web.js';
import * as fs from 'fs';
import axios from 'axios';
import ffmpeg from 'fluent-ffmpeg';
import gpt_tools from './chatgpt_funtions';

@Injectable()
export class OpenaiService {
  private readonly openai: OpenAIApi;

  constructor() {
    if (!process.env.OPENAI_API_KEY) throw new Error('OPENAI_API_KEY not set');

    this.openai = new OpenAIApi(
      new Configuration({
        apiKey: process.env.OPENAI_API_KEY,
      }),
    );
  }

  async getPointOpenAI(message: Message, client: Client, points: any) {
    const prompt = process.env.PROMPT_OPENAI_POINTS;

    const question = `Lista de pontos: ${JSON.stringify(
      points,
    )}. Identifique a responsta do usuário com base na lista de pontos e depois coloque as coordenadas do ponto em formato de float e responda apenas em português do Brasil. Pergunta do usuário: ${
      message.body
    }`;

    try {
      const response = await this.openai.createChatCompletion({
        model: 'gpt-4',
        messages: [
          { role: 'system', content: prompt },
          { role: 'user', content: question },
        ],
      });

      const pointResponse = response.data.choices[0].message?.content;

      const regex: RegExp =
        /-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?/gi;
      const match = pointResponse?.match(regex);
      // TODO: EMIT TO SOCKET
      // const socket = io('http://10.128.68.115:3000');
      if (match) {
        match.forEach((coordinateString) => {
          // Splitting the matched string into individual numbers
          const parts = coordinateString
            .split(',')
            .map((part) => parseFloat(part.trim()));
          const [x, y, z] = parts;
          // socket.emit('enqueue', { x, y, z });
          this.speechOpenAI(message, client, pointResponse);
        });
      } else {
        message.reply('Não consegui encontrar o ponto. Tente novamente.');
      }
    } catch (e) {
      console.error(e);
      client.sendMessage(message.from, JSON.stringify(e));
    }
  }

  async transcribeOpenAI(
    Message: Message,
    Client: Client,
    audioBuffer: Buffer,
  ): Promise<{ text: string; language: string }> {
    const url =
      process.env.TRANSCRIPTION_URL ||
      'https://api.openai.com/v1/audio/transcriptions';
    const language = process.env.TRANSCRIPTION_LANGUAGE || 'pt-BR';
    const oggPath = path.join(__dirname + '/media/', randomUUID() + '.ogg');
    const wavFilename = randomUUID() + '.wav';
    const wavPath = path.join(__dirname + '/media/', wavFilename);
    fs.writeFileSync(oggPath, audioBuffer);
    try {
      await this.convertOggToWav(oggPath, wavPath);
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

  async speechOpenAI(
    message: Message,
    client: Client,
    text: string | undefined,
  ): Promise<string> {
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
        },
      );
    } catch (e) {
      console.error(e);
      return '';
    } finally {
      const audioBase64 = Buffer.from(response.data, 'binary').toString(
        'base64',
      );
      const messageMedia = new MessageMedia('audio/ogg', audioBase64);
      await client.sendMessage(message.from, messageMedia, {
        sendAudioAsVoice: true,
      });
      return audioBase64;
    }
  }

  async convertOggToWav(oggPath: string, wavPath: string): Promise<void> {
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

  async interpretNewMessage() {
    return this.openai.createChatCompletion({
      model: 'gpt-3.5-turbo-1106',
      functions: gpt_tools,
      messages: [],
      temperature: 0.3,
    });
  }
}
