import { Inject, Injectable } from '@nestjs/common';
import { Chat, Message } from 'whatsapp-web.js';
import { UserService } from 'src/prisma/user.service';
import { AIService } from '../AI/AI.service';
import { UserDoesntExists } from 'src/prisma/user.service';
import { ToolService } from '../prisma/tool.service';
import { LocationService } from '../prisma/location.service';
import { OrderService } from '../prisma/order.service';
import parseCoordinates from './utils/parseCoordinates';
import { HandleUserService } from './handle-user.service';
import { HandleLeadService } from './handle-lead.service';
import { HandleAdminService } from './handle-admin.service';
import { TranscriptionService } from '../prisma/transcription.service';
import { Transcription, GPTFunctionCalling } from '@prisma/client';
import { GPTFunctionCallingService } from '../prisma/GPTFunctionCalling.service';

// interface para a mensagem que o GPT precisa no contexto
interface IParsedMessage {
  role: 'user' | 'assistant' | 'function';
  content?: string;
  // especificas para mensagens function
  name?: string;
}

export class MessageIsNotAudio extends Error {
  constructor(message: string = 'Message is not audio') {
    super(message);
    this.name = 'MessageIsNotAudio';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, MessageIsNotAudio);
    }
  }
}

interface CreateNewOrderArgs {
  from: number[];
  to: number[];
}

@Injectable()
export class HandlerService {
  private readonly functionMapping;
  private readonly readyTimestamp: number;

  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(AIService) private aiService: AIService,
    @Inject(ToolService) private toolService: ToolService,
    @Inject(LocationService) private locationService: LocationService,
    @Inject(OrderService) private orderService: OrderService,
    @Inject(HandleUserService) private handleUserService: HandleUserService,
    @Inject(HandleAdminService) private handleAdminService: HandleAdminService,
    @Inject(HandleLeadService) private handleLeadService: HandleLeadService,
    @Inject(TranscriptionService)
    private transcriptionService: TranscriptionService,
    @Inject(GPTFunctionCallingService)
    private gptFunctionCallingService: GPTFunctionCallingService,
  ) {
    this.functionMapping = {
      USER: this.handleUserService,
      ADMIN: this.handleAdminService,
      LEAD: this.handleLeadService,
    };

    this.readyTimestamp = Math.floor(+new Date() / 1000);
  }

  async handleIncomingMessage(message: Message): Promise<any> {
    console.log('\n\n[handleIncomingMessage] INCOMING NEW MESSAGE');
    let userData;
    try {
      userData = await this.userService.getUser(message.from);
    } catch (e) {
      if (e instanceof UserDoesntExists) {
        userData = null;
      } else {
        return 'Um erro aconteceu, contate um administrador.';
      }
    }

    console.log(
      '[handleIncomingMessage] sender:',
      message.from,
      ' -- role:',
      userData?.role,
    );

    const UserRole = (userData?.role as 'USER' | 'ADMIN' | 'LEAD') || 'LEAD';

    try {
      const trans = await this.handleNewMedia(message);

      if (trans) console.log(`[handleIncomingMessage] Transcription went fine`);
    } catch (e) {
      if (e instanceof MessageIsNotAudio)
        return 'Parece que não entendi o que você disse, poderia repetir?';

      console.log(`-> Erro ao tentar transcrever audio: ${e}`);
      return 'Um erro aconteceu, contate um administrador.';
    }

    const chat: Chat = await message.getChat();

    // Aparece o "digitando..." na conversa
    chat.sendStateTyping();

    const messages: Message[] = await chat.fetchMessages({ limit: 20 });

    const parsedMessages = await this.transformConversation(
      messages,
      this.readyTimestamp,
      await this.getFunctionCallsFromDB(message.from),
    );

    const toolCoordinates = await this.toolService.getAllTools();
    const locationCoordinates = await this.locationService.getAllLocations();

    const { parsedLocations, parsedTools } = parseCoordinates(
      toolCoordinates,
      locationCoordinates,
    );

    const res = await this.aiService.callGPT(
      UserRole,
      parsedTools,
      parsedLocations,
      parsedMessages as any,
      message.id.id,
      userData?.id,
    );

    // tira o "digitando..."
    chat.clearState();

    switch (res.type) {
      case 'message':
        console.log(
          '[handleNewMessage] GPT responded with a \x1b[31mMESSAGE\x1b[0m:\x1b[96m \n -> ' +
            res.message,
          '\x1b[0m',
        );
        return res.message;
      case 'function_call':
        try {
          console.log(
            '[handleNewMessage] GPT responded with a \x1b[31mFUNCTION CALL\x1b[0m! Calling:\x1b[96m',
            res.function,
            '\x1b[0m',
          );
          console.log('arguments: \x1b[35m', res.arguments, '\x1b[0m');
          const functionCallingRes = await this.functionMapping[UserRole][
            res.function
          ](message.from, res.arguments && res.arguments);

          // salva a function call no banco
          if (userData) {
            await this.gptFunctionCallingService.addFunctionCallingResponse(
              message.id.id,
              functionCallingRes,
            );
          }

          return functionCallingRes;
          // return await this[res.function](
          //   message.from,
          //   res.arguments && res.arguments,
          // );
        } catch (e) {
          return `${e}`;
        }
    }
  }

  private async transformConversation(
    chat: Message[],
    readyTimestamp: number,
    DBFunctionCalls: GPTFunctionCalling[],
  ): Promise<IParsedMessage[]> {
    const parsedMsgs: IParsedMessage[] = [];
    const nowTimestamp = Math.floor(+new Date() / 1000);

    let iterations = 0;

    for (const wppMessage of chat) {
      // checa se a mensagem foi mandada nas últimas 24h
      if (wppMessage.timestamp < nowTimestamp - 86400) continue;

      // checa se a mensagem foi enviada só depois do bot estar pronto
      if (wppMessage.timestamp < readyTimestamp) continue;
      //-----------------------
      let messageContent = '';

      if (wppMessage.hasMedia) {
        console.log(
          '[transformConversation] Arquivo de media encontrado. Tratando...',
        );
        const transcription: string | null =
          await this.transcriptionService.getTranscriptionFromMsgId(
            String(wppMessage.id.id),
          );

        if (!transcription)
          console.error(
            '\x1b[31m[transformConversation] ATENÇÃO! Mensagem vazia!\x1b[0m',
          );
        messageContent = transcription || '';
      } else messageContent = wppMessage.body;

      //-----------------------
      const role = wppMessage.id.fromMe ? 'assistant' : 'user';
      parsedMsgs.push({ role, content: messageContent });
      //*****************************

      // checa se a mensagem que está sendo analizada trigou uma função
      const FunctionCalling =
        DBFunctionCalls.find((obj) => obj.messageId === wppMessage.id.id) ||
        null;

      if (FunctionCalling) {
        console.log(
          '[transformConversation] \x1b[32mfound\x1b[0m function call, adding...',
        );
        const functionMessage = {
          role: 'function',
          content: FunctionCalling.functionResponse || '',
          name: FunctionCalling.functionName,
        };
        console.log(functionMessage);
        parsedMsgs.push(functionMessage as IParsedMessage);
      }

      //*****************************
      iterations++;
    }
    console.log(
      `[transformConversation] added ${iterations} messages to Context.`,
    );
    console.log(
      '[transformConversation] last message: \x1b[32m',
      parsedMsgs[parsedMsgs.length - 1].content,
      '\x1b[0m',
    );
    console.log(parsedMsgs);
    return parsedMsgs;
  }

  private async handleNewMedia(
    message: Message,
  ): Promise<void | Transcription> {
    if (!message.hasMedia) {
      console.log('[handleNewMedia] Message has no media');
      return;
    }
    const media = await message.downloadMedia();

    console.log('[handleNewMedia] Media type: ', media.mimetype);

    if (!media.mimetype.includes('ogg')) {
      console.log('[handleNewMedia] Message is not an audio');
      throw new MessageIsNotAudio();
    }

    const transcription = await this.aiService.speech2Text(media.data);

    console.log('[handleNewMedia] Transcription: ', transcription);

    if (transcription) {
      return await this.transcriptionService.insertNewTranscription(
        String(message.id.id),
        message.mediaKey,
        transcription,
      );
    }

    return;
  }

  private async getFunctionCallsFromDB(userPhone: string) {
    try {
      const { id } = await this.userService.getUser(userPhone);
      return await this.gptFunctionCallingService.getFunctionCallingByUserId(
        id,
      );
    } catch (e) {
      console.log(`-> Houve um erro ao tentar pegar as functionCalls: ${e}`);
      return [];
    }
  }
}