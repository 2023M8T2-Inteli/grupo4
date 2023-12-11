import { Inject, Injectable } from '@nestjs/common';
import { Message } from 'chatgpt';
import UserService from 'src/prisma/user.service';
import { WhatsappService } from 'src/whatsapp/whatsapp.service';
import { Client } from 'whatsapp-web.js';

@Injectable()
export class handleUserService {
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(WhatsappService) private whatsappService: WhatsappService,
  ) {}

  async handle(requestState: number, message: Message): Promise<void> {
    switch (requestState) {
      case 1:
        this.handleRequestMenu(message, this.whatsappClient);
        break;
      case 2:
        this.handleProcessRequest(message, this.whatsappClient);
        break;
      case 3:
        this.handleNewOrder(message, this.whatsappClient);
        break;
      case 4:
        this.handleStatusOrder(message, this.whatsappClient);
        break;
      case 5:
        this.handleCancelOrder(message, this.whatsappClient);
        break;
      case 6:
        this.handleUpdateName(message, this.whatsappClient);
        break;
      default:
        break;
    }
  }
}