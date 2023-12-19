import { Inject, Injectable } from '@nestjs/common';
import { Chat, Message } from 'whatsapp-web.js';
import { UserService } from 'src/prisma/user.service';
import { AIService } from '../AI/AI.service';
import transformConversation from './utils/transformConversation';
import { UserDoesntExists } from 'src/prisma/user.service';
import { io, Socket } from 'socket.io-client';
import { ToolService } from '../prisma/tool.service';
import { LocationService } from '../prisma/location.service';
import { OrderService } from '../prisma/order.service';
import parseCoordinates from './utils/parseCoordinates';
import { HandleUserService } from './handle-user.service';
import { HandleLeadService } from './handle-lead.service';
import { HandleAdminService } from './handle-admin.service';
import { TranscriptionService } from '../prisma/transcription.service';
import { Transcription } from '@prisma/client';

interface ParsedMessage {
  role: 'user' | 'assistant';
  content: string;
}

@Injectable()
export class HandlerService {
  private readonly sioClient: Socket;
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
  ) {
    this.sioClient = io(process.env.SOCKET_URL || '');

    this.functionMapping = {
      USER: this.handleUserService,
      ADMIN: this.handleAdminService,
      LEAD: this.handleLeadService,
    };

    this.readyTimestamp = Math.floor(+new Date() / 1000);
  }

  async handleIncomingMessage(message: Message): Promise<any> {
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

    console.log('-> sender:', message.from, ' -- role:', userData?.role);

    const UserRole = (userData?.role as 'USER' | 'ADMIN' | 'LEAD') || 'LEAD';

    const trans = await this.handleNewMedia(message);

    if (!trans)
      return 'Parece que não entendi o que você disse, poderia repetir?';

    const chat: Chat = await message.getChat();

    // Aparece o "digitando..." na conversa
    chat.sendStateTyping();

    const messages: Message[] = await chat.fetchMessages({ limit: 20 });

    const parsedMessages = await this.transformConversation(
      messages,
      this.readyTimestamp,
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
      parsedMessages,
    );

    // tira o "digitando..."
    chat.clearState();

    switch (res.type) {
      case 'message':
        return res.message;
      case 'function_call':
        try {
          console.log(res.function);
          return await this.functionMapping[UserRole][res.function](
            message.from,
            res.arguments && res.arguments,
          );
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
  ): Promise<ParsedMessage[]> {
    const parsedMsgs: ParsedMessage[] = [];
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
        const transcription: string | null =
          await this.transcriptionService.getTranscriptionFromMsgId(
            String(wppMessage.id.id),
          );

        if (!transcription)
          console.error(
            '---- ATENÇÃO! Serviço de stt retornou uma mensagem vazia!',
          );

        messageContent = transcription || '';
      } else messageContent = wppMessage.body;

      //-----------------------
      const role = wppMessage.id.fromMe ? 'assistant' : 'user';
      parsedMsgs.push({ role, content: messageContent });
      iterations++;
    }
    console.log(`-> Added ${iterations} messages to Context.`);
    console.log('-> last message: ', parsedMsgs[parsedMsgs.length - 1].content);
    return parsedMsgs;
  }

  private async handleNewMedia(
    message: Message,
  ): Promise<void | Transcription> {
    if (!message.hasMedia) {
      console.log('-> Message has no media');
      return;
    }
    const media = await message.downloadMedia();

    console.log('-> Media type: ', media.mimetype);

    if (!media.mimetype.includes('ogg')) {
      console.log('-> Message is not an audio');
      return;
    }

    const transcription = await this.aiService.speech2Text(media.data);

    console.log('-> Transcription: ', transcription);

    if (transcription) {
      return await this.transcriptionService.insertNewTranscription(
        String(message.id.id),
        message.mediaKey,
        transcription,
      );
    }

    return;
  }
}
