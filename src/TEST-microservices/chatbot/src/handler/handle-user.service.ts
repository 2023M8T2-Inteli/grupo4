// import { Inject, Injectable } from '@nestjs/common';
// import UserService from 'src/prisma/user.service';
// import { WhatsappService } from 'src/whatsapp/whatsapp.service';
// import { Message } from 'whatsapp-web.js';
// import { HandlerBase } from './classes/base';
// import { io, Socket } from 'socket.io-client';
// import { OrderService } from '../prisma/order.service';
//
// @Injectable()
// export class handleUserService extends HandlerBase {
//   private readonly sioClient: Socket;
//   constructor(
//     @Inject(UserService) private userService: UserService,
//     @Inject(WhatsappService) private whatsappService: WhatsappService,
//     @Inject(OrderService) private orderService: OrderService,
//   ) {
//     super(whatsappService);
//     this.sioClient = io(process.env.SOCKET_URL || '');
//   }
//
//   public handleNewOrder(toolCoords: number[], arriveCoords: number[]): string {
//     if (toolCoords.length != 3 || arriveCoords.length != 3) {
//       return 'Coords in the wrong format';
//     }
//
//     this.sioClient.emit('enqueue', {
//       x: toolCoords[0],
//       y: toolCoords[1],
//       z: toolCoords[2],
//     });
//
//     this.sioClient.emit('enqueue', {
//       x: arriveCoords[0],
//       y: arriveCoords[1],
//       z: arriveCoords[2],
//     });
//
//     return 'Coordenas enviadas para o robô';
//   }
//
//   public async handleStatusOrderById(
//     user: string,
//     _order: number,
//   ): Promise<string> {
//     const order = await this.orderService.getOrderByCode(user, _order);
//     if (order) {
//       return (
//         '*Pedido:* ' +
//         order.code +
//         '\n*Status:* ' +
//         order.type +
//         '\n*Data:* ' +
//         order.createdAt
//       );
//     }
//     return 'Ordem não encontrada';
//   }
//
//   public async handleStatusLastOrder(user: string): Promise<string> {
//     const order = await this.orderService.getLastOrder(user);
//     if (order) {
//       return (
//         '*Pedido:* ' +
//         order.code +
//         '\n*Status:* ' +
//         order.type +
//         '\n*Data:* ' +
//         order.createdAt
//       );
//     }
//     return 'Ordem não encontrada';
//   }
//
//   public async handleOrdersMessage(userPhone: string) {
//     try {
//       const orders = await this.orderService.getOpenOrder(userPhone);
//       if (orders && orders.length > 0) {
//         const messageOrders = [];
//         for (const order of orders) {
//           messageOrders.push(
//             '*Pedido:* ' +
//               order.code +
//               '\n*Status:* ' +
//               order.type +
//               '\n*Data:* ' +
//               order.createdAt,
//           );
//         }
//         return messageOrders;
//         // userService.updateRequestUser(message.from, 1);
//       } else {
//         return 'No momento você não possui pedidos em aberto.';
//       }
//     } catch (error: any) {
//       console.error('An error occured', error);
//       return (
//         'An error occured, please contact the administrator. (' +
//         error.message +
//         ')'
//       );
//     }
//   }
//
//   public async handleSendAdminContact(message: Message) {
//     try {
//       const user = await this.userService.getAdmin();
//       if (user) {
//         const contact = await this.whatsappService.getContactById(
//           user?.cellPhone,
//         );
//         this.whatsappService.sendMessage(message.from, contact);
//
//         return 'Contato enviado para o usuário!';
//       } else {
//         return 'No momento não temos atendentes disponíveis, por favor, tente novamente mais tarde.';
//       }
//     } catch (error: any) {
//       console.error('An error occured', error);
//       return (
//         'An error occured, please contact the administrator. (' +
//         error.message +
//         ')'
//       );
//     }
//   }
//
//   public async handleCancelOrder(
//     userPhone: string,
//     order_: number,
//   ): Promise<string> {
//     try {
//       const user = await this.userService.getUser(userPhone);
//
//       if (user) {
//         const order = await this.orderService.cancelOrder(userPhone, order_);
//         if (!order) {
//           return 'Não foi possível cancelar a ordem';
//         }
//
//         return 'Ordem cancelada com sucesso!';
//       }
//     } catch (e) {
//       return `Error: ${e}`;
//     }
//   }
//
//   public async handleUpdateName(userPhone: string, dataToUpdate: any) {
//     try {
//       const user = await this.userService.getUser(userPhone);
//
//       if (!user) {
//         return 'Usário não encontrado com esse nome';
//       }
//
//       await this.userService.updateAccountUser(dataToUpdate);
//
//       return 'Dados do usuário atualizado com sucesso!';
//     } catch (e) {
//       return `Error: ${e}`;
//     }
//   }
// }
