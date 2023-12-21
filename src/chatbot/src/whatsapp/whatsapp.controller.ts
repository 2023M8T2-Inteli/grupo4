import { Controller, Get, Inject } from '@nestjs/common';
import { WhatsappService } from './whatsapp.service';
import { MessagePattern } from '@nestjs/microservices';

@Controller()
export class WhatsappController {
  constructor(
    @Inject(WhatsappService) private readonly whatsappService: WhatsappService,
  ) {}

  @Get("qrcode")
  getQrCode(): any {
    return this.whatsappService.getQrCodeUrl();
  }

  @Get("chatbotHealthCheck")
  chatbotHealthCheck(): any {
    return this.whatsappService.getQrCodeUrl();
  }

}
