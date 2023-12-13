import { Inject, Injectable } from '@nestjs/common';
import { Chat, Message } from 'whatsapp-web.js';
import { UserService } from 'src/prisma/user.service';
import { AIService } from '../AI/AI.service';
import transformConversation from './utils/transformConversation';

@Injectable()
export class HandlerService {
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(AIService) private aiService: AIService,
  ) {}

  async handleIncomingMessage(message: Message): Promise<void> {
    const userData = await this.userService.getUser(message.from);

    const chat: Chat = await message.getChat();

    const messages: Message[] = await chat.fetchMessages({ limit: 20 });

    const parsedMessages = transformConversation(messages);

    console.log(chat);

    const res = await this.aiService.callGPT(
      (userData?.role as 'USER' | 'ADMIN' | 'LEAD') || 'LEAD',
      `
        Brahma - [0,5]
        Skol - [3,5]
        Heigenen - [-345, 234]
        Salgadinhos - [-23, 120]
        `,
      parsedMessages,
    );
    console.log(res);
  }
}
