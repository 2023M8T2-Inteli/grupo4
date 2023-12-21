import { Inject, Injectable } from '@nestjs/common';
import { randomUUID } from 'crypto';
import { TextToSpeechClient } from '@google-cloud/text-to-speech';
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
import { GPTFunctionCallingService } from '../prisma/GPTFunctionCalling.service';

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
  private readonly ttsClient: TextToSpeechClient;
  public readonly vectorizedData: any;

  constructor(
    @Inject(GPTFunctionCallingService)
    private gptFunctionCallingService: GPTFunctionCallingService,
  ) {
    if (!process.env.OPENAI_API_KEY) throw new Error('OPENAI_API_KEY not set');

    this.openai = new OpenAIApi(
      new Configuration({
        apiKey: process.env.OPENAI_API_KEY,
      }),
    );
    let credentials: any;
    try {
      credentials = JSON.parse(
        fs.readFileSync(path.join(__dirname, 'googlespeech.json'), 'utf-8'),
      );
    } catch (e) {
      credentials = {
        type: process.env.TYPE,
        project_id: process.env.PROJECT_ID,
        private_key_id: process.env.PRIVATE_KEY_ID,
        private_key: process.env.PRIVATE_KEY?.replace(/\\n/g, '\n'),
        client_email: process.env.CLIENT_EMAIL,
        client_id: process.env.CLIENT_ID,
        auth_uri: process.env.AUTH_URI,
        token_uri: process.env.TOKEN_URI,
        auth_provider_x509_cert_url: process.env.AUTH_PROVIDER_X509_CERT_URL,
        client_x509_cert_url: process.env.CLIENT_X509_CERT_URL,
        universe_domain: process.env.UNIVERSE_DOMAIN,
      };
    }
    if (!credentials) {
      throw new Error('GOOGLE_APPLICATION_CREDENTIALS not set');
    }

    this.ttsClient = new TextToSpeechClient({
      credentials,
    });
  }

  // função principal para gerar uma resposta pelo GPT
  callGPT = async (
    userRole: 'ADMIN' | 'USER' | 'LEAD' = 'LEAD',
    toolsCordinates: string,
    locationCoordinates: string,
    chatHistory: ChatHistory[],
    messageId: string,
    userId: string,
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
      temperature: 0.6,
    });
    if (res.data.choices[0].message?.function_call?.name) {
      if (userId) {
        await this.gptFunctionCallingService.insertNewFunctionCalling(
          res.data.id,
          JSON.stringify(res.data.choices[0].message),
          res.data.choices[0].message.function_call.name,
          messageId,
          userId,
        );
      }
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
        'pt-BR',
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

  async text2Speech(text: string) {
    let response;
    try {
      response = await this.ttsClient.synthesizeSpeech({
        input: { text },
        voice: {
          languageCode: 'pt-BR',
          name: 'pt-BR-Neural2-B',
        },
        audioConfig: { audioEncoding: 'MP3', pitch: 13, speakingRate: 1.2 },
      });

      const audioBase64 = Buffer.from(
        response[0].audioContent,
        'binary',
      ).toString('base64');

      return audioBase64;
    } catch (e) {
      console.log(`-> Error: ${e}`);
      return;
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

  // METODOS ESPECIFICOS PARA A INTERFACE WEB ---------------------------------------

  filterJson(inputJson) {
    return `
      ferramenta: ${inputJson?.tool?.name || ''},
      requisitante/usuário:  ${inputJson?.user?.name || ''},
      local/destino/ponto:  ${inputJson?.point?.name || 0},
      data de criação:  ${inputJson?.createdAt || ''},
      -------------------------------------------------
    `;
  }

  parseOrders(inputJson) {
    let result = '';
    inputJson.forEach((item) => {
      result += item;
    });
    return result
  }

  // Função que cónstroi o contexto para o GPT da interface
  // antes chamada de buildContext
  async buildInterfaceSystemMessage() {
    try {
      const response = await axios.get(
        process.env.NEXT_PUBLIC_BACKEND + '/orders/queue',
      );

      let now = ''
      response.data.forEach(item => {

        if(item.type === "Collecting" || item.type === "To confirm"){
          now = item
        }
      })

      let queue = []
      response.data.forEach(item => {

        if(item.type === "In Progress"){
          queue.push(item)
        }
      })
      const data = {
        now,
        queue,
        history: [],
      };
      const history = await axios.get( 
        process.env.NEXT_PUBLIC_BACKEND + '/orders/history',
      );
      data['history'] = history.data;

      // Use Array.map to apply filterJson to each item in the array
      const filteredArray = data?.queue.map((item) => this.filterJson(item));

      const filteredHistory = data?.history.map((item) =>
        this.filterJson(item),
      );

      
  
        
    

      return {
        role: 'system',
        content: `
      Você é um assistente para delivery de peças do almoxarifado na Ambev. Seu nome é Vallet. Responda as perguntas de forma simpática e divertida. Na primeira resposta, se apresente e diga o que pode fazer. Seja conciso. O "point" nos dados é o lugar para onde o itens será entregue, onde se encontra a pessoa que fez pedido. Presta atenção para não confundir fila com histórico. Se a pessoa perguntar o que falta, é a fila, se ela perguntar o que já foi entregue, é o histórico. Não responda com emojis. Só fale em português, não importa a língua com que falarem com você. Se algo não fizer sentido, diga que não entendeu.
      
      PEDIDO SENDO EXECUTADO AGORA: ${this.filterJson(data?.now)}
      Nº DE ITENS NA FILA (PEDIDOS QUE FALTAM SER ENTREGUES): ${
        data?.queue.length || 'não há pedidos na fila'
      } 
      FILA (PEDIDOS QUE FALTAM SER ENTREGUES): ${this.parseOrders(
        filteredArray,
      )}
      Nº DE ITENS NO HISTÓRICO (PEDIDOS QUE JÁ FORAM ENTREGUES): ${data?.history
        .length}
      HISTÓRICO:${this.parseOrders(filteredHistory)}
      `,
      };
    } catch (error) {
      console.log(error);
    }
  }

  //função que pega a emoção de um texto para a interface
  async getMessageEmotion(text: string) {
    const res = await this.openai.createChatCompletion({
      model: 'gpt-3.5-turbo-1106',
      messages: [
        {
          role: 'system',
          content:
            'Leia o texto abaixo e defina a emoção principal dele entre happy, neutral, superhappy, e sad. RESPONDA APENAS COM UMA PALAVRA: happy, supperhappy, neutral ou sad. Se o usuário elogiar ou agradecer, deve ser superhappy. Se a resposta for um pedido de desculpas, deve ser triste.: ' +
            text,
        },
      ],
    });
    return res.data.choices[0].message.content;
  }

  //callGPT especifico para a conversa da interface
  async callGPTInterface(systemMessage: any, messages: Array<any>) {
    const res = await this.openai.createChatCompletion({
      model: 'gpt-3.5-turbo-1106',
      messages: [systemMessage, ...messages],
    });
    return res.data.choices[0].message.content;
  }
}
