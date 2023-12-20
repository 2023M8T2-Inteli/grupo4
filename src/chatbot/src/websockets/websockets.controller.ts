import { Controller, Get, Inject } from '@nestjs/common';
import { WebsocketService } from './websocket.service';

@Controller('websockets')
export class WebsocketController {
  constructor(@Inject(WebsocketService) private readonly websocketService: WebsocketService) {}

  @Get('/battery')
  getBattery() {
    return this.websocketService.getBattery();
  }
}
