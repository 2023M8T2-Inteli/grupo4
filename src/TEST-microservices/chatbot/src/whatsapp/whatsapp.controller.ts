import { Controller, Inject } from '@nestjs/common';
import { WhatsappService } from './whatsapp.service';
import { MessagePattern } from '@nestjs/microservices';

@Controller()
export class WhatsappController {
  constructor(
    @Inject(WhatsappService) private readonly whatsappService: WhatsappService,
  ) {}

  @MessagePattern({ cmd: 'qrcode' })
  getQrCode(): string {
    return this.whatsappService.getQrCodeUrl();
  }

  // retorna a página fake do whatsapp web para escanear o qrcode
  // @MessagePattern({ cmd: 'page' })
  // getPage() {
  //   return this.whatsappService.getPage(data);
  // }
}
