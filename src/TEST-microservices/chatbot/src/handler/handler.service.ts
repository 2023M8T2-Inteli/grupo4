import { Inject, Injectable } from '@nestjs/common';
import { Client, Events, LocalAuth, Message } from 'whatsapp-web.js';
import qrcode from 'qrcode';
import { PrismaService } from 'src/prisma/prisma.service';
import UserService from 'src/prisma/user.service';
import { WhatsappService } from '../whatsapp/whatsapp.service';import { handleAdminService } from './handle-admin.service';
import { handleLeadService } from './handle-lead.service';
import { handleUserService } from './handle-user.service';

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
    
    // TODO: message validation

    const userData = await this.userService.getUser(message.from);

    if (message.body == "!sair" || message.body == "!Sair") {
        this.whatsappService.sendMessage(message.from, "Até mais!");
        this.userService.updateRequestUser(message.from, 1);
        return;
    }

    if (userData?.role?.includes("USER")) {
        let requestState = userData?.requestState;
        this.handleUserService.handle(requestState, message)
    } 
    if(userData?.role?.includes("ADMIN")){
        let requestState = userData?.requestState;
        this.handleAdminService.handle(requestState, message)
    }
    if(userData?.role?.includes("LEAD") || userData == null) {
        this.handleLeadService.handle(userData, message)
    }
}

}
