import { Injectable } from '@nestjs/common';
import { randomUUID } from 'crypto';
import { Readable } from 'stream';
import { Configuration, OpenAIApi } from 'openai';
import * as path from 'path';
import FormData from 'form-data';
import { Client, Message, MessageMedia } from 'whatsapp-web.js';
import * as fs from 'fs';
import axios from 'axios';
import ffmpeg from 'fluent-ffmpeg';
import { generateLLMSystemMessages } from './chatgpt_funtions';
import * as os from 'os';
import * as crypto from 'crypto';

interface ChatHistory {
  role: 'user' | 'system' | 'assistant' | 'function';
  content: string;
}

export interface GPTResponseFunctionCall {
  type: 'function_call';
  function: string;
  arguments: object;
}

export interface GPTResponseMessage {
  type: 'message';
  message: string;
}
@Injectable()
export class AIService {
  private readonly openai: OpenAIApi;
  public readonly vectorizedData: any;

  constructor() {
    if (!process.env.OPENAI_API_KEY) throw new Error('OPENAI_API_KEY not set');

    this.openai = new OpenAIApi(
      new Configuration({
        apiKey: process.env.OPENAI_API_KEY,
      }),
    );
  }

  // função principal para gerar uma resposta pelo GPT
  callGPT = async (
    userRole: 'ADMIN' | 'USER' | 'LEAD' = 'LEAD',
    toolsCordinates: string,
    locationCoordinates: string,
    chatHistory: ChatHistory[],
  ): Promise<GPTResponseFunctionCall | GPTResponseMessage> => {
    const { system_message, gpt_tools } = generateLLMSystemMessages(
      userRole,
      toolsCordinates,
      locationCoordinates,
    );

    const res = await this.openai.createChatCompletion({
      functions: gpt_tools,
      messages: [{ role: 'system', content: system_message }, ...chatHistory],
      model: 'gpt-4',
      temperature: 0.8,
    });
    console.log(res.data.choices[0]);
    if (res.data.choices[0].message?.function_call?.name) {
      return {
        type: 'function_call',
        function: res.data.choices[0].message.function_call.name,
        arguments: JSON.parse(
          res.data.choices[0].message.function_call.arguments,
        ),
      };
    } else {
      return {
        type: 'message',
        message: res.data.choices[0].message.content,
      };
    }
  };

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

  async speech2Text(audio: any): Promise<void | string> {
    const tempFilePath = path.join(
      os.tmpdir(),
      `${crypto.randomBytes(10).toString('hex')}.ogg`,
    );

    fs.writeFileSync(tempFilePath, Buffer.from(audio, 'base64'));

    const audioStream = fs.createReadStream(tempFilePath);

    try {
      const res = await this.openai.createTranscription(
        audioStream as any,
        'whisper-1',
      );

      return res.data.text;
    } catch (e) {
      console.log(`-> Error: ${e}`);
      return;
    } finally {
      fs.unlink(tempFilePath, (err) => {
        if (err) console.error('Error deleting temporary file:', err);
      });
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
}
