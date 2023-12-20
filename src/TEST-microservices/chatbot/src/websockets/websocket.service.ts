import { Inject, Injectable, forwardRef } from '@nestjs/common';
import { WebSocketServer } from '@nestjs/websockets';
import { Server } from 'http';
import { WebsocketGateway } from './websocket.gateway';
import { OrderService } from 'src/prisma/order.service';
import { WhatsappService } from 'src/whatsapp/whatsapp.service';
import { UserService } from 'src/prisma/user.service';

interface Itask {
  id: string;
  type: 'GRAB' | 'DROP';
  x: number;
  y: number;
}

@Injectable()
export class WebsocketService {
  private lastBatteryReading: number;

  constructor(
    @Inject(forwardRef(() => WebsocketGateway))
    private readonly websocketGateway: WebsocketGateway,
    @Inject(OrderService) private readonly orderService: OrderService,
    @Inject(forwardRef(() => WhatsappService))
    private readonly whatsappService: WhatsappService,
    @Inject(UserService) private readonly userService: UserService,
  ) {
    this.lastBatteryReading = 99;
  }

  taskFeedback(id: string, type: 'GRAB' | 'DROP') {
    this.websocketGateway.server.emit('hello', { id, type });
  }

  addPointToQueue(data: Itask) {
    this.websocketGateway.server.emit('/enqueue', JSON.stringify(data));
  }

  emergencyStop(data: { emergency_stop: number }) {
    console.log(
      `[WebsocketService] Emitting \x1b[31m${
        data.emergency_stop ? 'STOP' : 'CONTINUE'
      }\x1b[0m`,
    );
    this.websocketGateway.server.emit('/emergency_stop', JSON.stringify(data));
  }

  async handleGrabTask(id: string) {
    try {
      if (!Number(id)) throw new Error('Invalid format ID');

      const updatedOrder = await this.orderService.updateOrderStatusByCode(
        Number(id),
        'Collecting',
      );

      const { cellPhone, name } = await this.userService.getUserById(
        updatedOrder.userId,
      );

      const message = `Olá ${name}!
      \n Venho para trazer novas informações sobre o pedido: ${updatedOrder.code} 😀
      \n Estamos a caminho de buscar a sua peça nesse exato instante! Assim que chegarmos no destino iremos te avisar! 🫡`;

      this.whatsappService.sendMessage(cellPhone, message);
    } catch (e) {
      console.log(`-> Error ao tentar lidar com uma GRAB task: ${e}`);
    }
  }

  async handleDropTask(id: string) {
    try {
      if (!Number(id)) throw new Error('Invalid format ID');

      const updatedOrder = await this.orderService.updateOrderStatusByCode(
        Number(id),
        'To confirm',
      );

      const { cellPhone, name } = await this.userService.getUserById(
        updatedOrder.userId,
      );

      const message = `Olá ${name}!
      \n Venho para trazer novas informações sobre o pedido: ${updatedOrder.code} 😀
      \n Estamos a caminho do destinho nesse exato instante! Para finalizar o pedido, precisamos que você confirme a entrega! Assim que chegarmos, por favor, confirme a entrega enviando uma mensagem aqui no chat mesmo! 🥳`;

      this.whatsappService.sendMessage(cellPhone, message);
    } catch (e) {
      console.log(`-> Error ao tentar lidar com uma DROP task: ${e}`);
    }
  }

  newBatteryReading(battery: number) {
    this.lastBatteryReading = battery;
  }

  getBattery() {
    return this.lastBatteryReading;
  }
}
