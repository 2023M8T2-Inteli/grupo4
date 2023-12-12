import { Inject, Injectable } from '@nestjs/common';
import { Client, Events, LocalAuth, Message } from 'whatsapp-web.js';
import qrcode from 'qrcode';
import { PrismaService } from 'src/prisma/prisma.service';
import UserService from 'src/prisma/user.service';
import { WhatsappService } from '../whatsapp/whatsapp.service';
import { handleAdminService } from './handle-admin.service';
import { handleLeadService } from './handle-lead.service';
import { handleUserService } from './handle-user.service';
import {validate_message, check_out} from "./utils/validate_msg";

@Injectable()
export class HandlerService {
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(WhatsappService) private whatsappService: WhatsappService,

    @Inject(handleAdminService) private handleAdminService: handleAdminService,
    @Inject(handleLeadService) private handleLeadService: handleLeadService,
    @Inject(handleUserService) private handleUserService: handleUserService,
  ) {}

  async handleIncomingMessage(message: Message): Promise<void> {
    if (
      !(await validate_message(message, this.whatsappService.botReadyTimestamp))
    ) {
      return;
    }
    // checka se a mensagem enviada é "!sair"
    if (await check_out(message, this.whatsappService, this.userService)) {
      return;
    }

    const userData = await this.userService.getUser(message.from);

    switch (userData?.role) {
      case 'USER': {
        const requestState = userData?.requestState;
        this.handleUserService.handle(requestState, message);
        break;
      }
      case 'ADMIN': {
        const requestState = userData?.requestState;
        this.handleAdminService.handle(requestState, message);
        break;
      }
      default: {
        this.handleLeadService.handle(userData, message);
        break;
      }
    }
  }
}
