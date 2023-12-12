import { Inject, Injectable } from '@nestjs/common';
import UserService from 'src/prisma/user.service';
import { WhatsappService } from 'src/whatsapp/whatsapp.service';
import { Client, Message } from 'whatsapp-web.js';
import { HandlerBase } from './classes/base';
import { io, Socket } from 'socket.io-client';

type ActionDict = { [key: string]: (message: Message, client: Client) => any };

@Injectable()
export class handleUserService extends HandlerBase {
  private readonly actionDict: ActionDict;
  private readonly sioClient: Socket;
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(WhatsappService) private whatsappService: WhatsappService,
    @Inject(OrderService) private orderService: OrderService,
  ) {
    super(whatsappService);
    this.sioClient = io(process.env.SOCKET_URL || '');
    this.actionDict = {
      openOrders: this.sendOpenOrders,
      cancelOrder: this.sendCancelOrder,
      contact: this.sendContact,
      changeName: this.sendChangeName,
    };
  }

  public handleNewOrder(toolCoords: number[], arriveCoords: number[]): string {
    if (toolCoords.length != 3 || arriveCoords.length != 3) {
      return 'Coords in the wrong format';
    }

    this.sioClient.emit('enqueue', {
      x: toolCoords[0],
      y: toolCoords[1],
      z: toolCoords[2],
    });

    this.sioClient.emit('enqueue', {
      x: arriveCoords[0],
      y: arriveCoords[1],
      z: arriveCoords[2],
    });

    return 'Coordenas enviadas para o robô';
  }

  public async handleStatusOrderById(
    user: string,
    _order: number,
  ): Promise<string> {
    const order = await this.orderService.getOrderById(user, _order);
    if (order) {
      return (
        '*Pedido:* ' +
        order.code +
        '\n*Status:* ' +
        order.type +
        '\n*Data:* ' +
        order.createdAt
      );
    }
    return 'Ordem não encontrada';
  }

  public async handleStatusLastOrder(user: string): Promise<string> {
    const order = await this.orderService.getLastOrder(user);
    if (order) {
      return (
        '*Pedido:* ' +
        order.code +
        '\n*Status:* ' +
        order.type +
        '\n*Data:* ' +
        order.createdAt
      );
    }
    return 'Ordem não encontrada';
  }

  private sendNewOrder(message: Message) {
    try {
      message.reply('Certo, você deseja solicitar uma nova peça.');
      // this.userService.updateRequestUser(message.from, 3);
      message.reply(
        'Você pode me dizer qual peça deseja solicitar ou situação que está enfrentando?',
      );
      this.whatsappService.sendMessage(
        message.from,
        'Se preferir, você pode me enviar um áudio com a sua solicitação.',
      );
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  private sendStatusOrder(message: Message) {
    try {
      message.reply('Certo, você deseja acompanhar o status de um pedido.');
      // this.userService.updateRequestUser(message.from, 4);
      message.reply(
        'Por favor, digite o número do pedido que deseja acompanhar.',
      );
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  private sendOpenOrders(message: Message) {
    try {
      message.reply('Certo, você deseja acompanhar seus pedidos em aberto.');
      this.whatsappService.sendMessage(
        message.from,
        'Certo. Aguarde um momento por favor.',
      );
      this.generateOrdersMessage(message);
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  private async generateOrdersMessage(message: Message) {
    try {
      const orders = await this.orderService.getOpenOrder(message);
      if (orders && orders.length > 0) {
        for (const order of orders) {
          message.reply(
            '*Pedido:* ' +
              order.code +
              '\n*Status:* ' +
              order.type +
              '\n*Data:* ' +
              order.createdAt,
          );
          await this.delay(1000);
        }
        // userService.updateRequestUser(message.from, 1);
      } else {
        message.reply('No momento você não possui pedidos em aberto.');
      }
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  private sendCancelOrder(message: Message) {
    try {
      message.reply('Certo, você deseja cancelar um pedido.');
      // userService.updateRequestUser(message.from, 5);
      message.reply(
        'Por favor, digite o número do pedido que deseja cancelar.',
      );
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  private async sendContact(message: Message) {
    try {
      message.reply('Certo, você deseja falar com um atendente.');
      const contact = await this.getAdminContact(message);
      if (contact) {
        this.whatsappService.sendMessage(message.from, contact);
      }
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  private async getAdminContact(message: Message) {
    try {
      const user = await this.userService.getAdmin();
      if (user) {
        return await this.whatsappService.getContactById(user?.cellPhone);
      } else {
        message.reply(
          'No momento não temos atendentes disponíveis, por favor, tente novamente mais tarde.',
        );
      }
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  private sendChangeName(message: Message) {
    try {
      message.reply('Certo, você deseja alterar seu nome cadastrado.');
      // userService.updateRequestUser(message.from, 6);
      this.whatsappService.sendMessage(
        message.from,
        'Por favor, digite seu nome completo, por favor.',
      );
    } catch (error: any) {
      console.error('An error occured', error);
      message.reply(
        'An error occured, please contact the administrator. (' +
          error.message +
          ')',
      );
    }
  }

  async handle(requestState: number, message: Message): Promise<void> {
    switch (requestState) {
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
