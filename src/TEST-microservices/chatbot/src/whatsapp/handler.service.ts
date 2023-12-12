import { Inject, Injectable } from '@nestjs/common';
import { Client, Events, LocalAuth, Message } from 'whatsapp-web.js';
import constants from './constants';
import qrcode from 'qrcode';
import { PrismaService } from 'src/prisma/prisma.service';
import UserService from 'src/prisma/user.service';
import { WhatsappService } from './whatsapp.service';

@Injectable()
export class HandlerService {
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(PrismaService) private prisma: PrismaService,
    @Inject(WhatsappService) private whatsappService: WhatsappService,
  ) {}
  async handleIncomingMessage(message: Message): Promise<void> {
    // TODO: message validation

    const userData = await this.userService.getUser(message.from);

    if (message.body == '!sair' || message.body == '!Sair') {
      this.whatsappService.sendMessage(message.from, 'Até mais!');
      this.userService.updateRequestUser(message.from, 1);
      return;
    }

    if (userData?.role?.includes('USER')) {
      let requestState = userData?.requestState;
      const requestUserHandler = new RequestUserHandler(
        this.whatsappClient,
        this.userService,
      );
      requestUserHandler.handle(requestState, message);
    }
    if (userData?.role?.includes('ADMIN')) {
      let requestState = userData?.requestState;
      const requestAdminHandler = new RequestAdminHandler(
        this.whatsappClient,
        this.userService,
      );
      requestAdminHandler.handle(requestState, message);
    }
    if (userData?.role?.includes('LEAD') || userData == null) {
      let requestLeadHandler = new RequestLeadHandler(
        this.whatsappClient,
        this.userService,
      );
      requestLeadHandler.handle(message, userData);
    }
  }

  async handleUserMessage() {}
}
