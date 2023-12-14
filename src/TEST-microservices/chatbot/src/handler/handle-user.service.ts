import { Inject, Injectable } from '@nestjs/common';
import { io, Socket } from 'socket.io-client';
import { Role } from '@prisma/client';
import { UserService } from '../prisma/user.service';
import { OrderService } from '../prisma/order.service';
import { LocationService } from '../prisma/location.service';
import { ToolService } from '../prisma/tool.service';

interface CreateNewOrderArgs {
  from: number[];
  to: number[];
}

@Injectable()
export class HandleUserService {
  protected readonly sioClient: Socket;

  constructor(
    @Inject(UserService) protected userService: UserService,
    @Inject(OrderService) protected orderService: OrderService,
    @Inject(LocationService) protected locationService: LocationService,
    @Inject(ToolService) protected toolService: ToolService,
  ) {
    this.sioClient = io(process.env.SOCKET_URL || '');
  }

  async handleNewOrder(userPhone: string, args: CreateNewOrderArgs) {
    const from = args?.from;
    const to = args?.to;

    if (from?.length === 2 && to?.length === 2) {
      return await this.generateNewOrder(userPhone, from, to);
    }

    return `Não foi possível processar o seu pedido. As seguintes informações estão faltando: ${
      from?.length !== 2 && '\n - origem do pedido'
    } ${to?.length !== 2 && '\n - destino do pedido'}. \n  😀`;
  }

  protected async generateNewOrder(
    userPhone: string,
    from: number[],
    to: number[],
  ) {
    if ((await this.userService.getUserRole(userPhone)) === Role.LEAD)
      return 'Ainda não é possível realizar pedidos, por favor aguarde um administrador liberar seu acesso.';

    if (!(await this.toolService.coordsExists(from)))
      return 'Infelizmente não temos esse produto em nosso estoque. Gostaria de fazer outro pedido?';

    if (!(await this.locationService.locationExists(to)))
      return 'Infelizmente não conseguimos entregar nesse endereço. Gostaria de fazer outro pedido?';

    const toolId = await this.toolService.getToolIdByCoords(from);
    const locationId = await this.locationService.getLocationIdByCoords(to);

    try {
      const order = await this.orderService.createOrder(
        userPhone,
        toolId,
        locationId,
      );

      this.sioClient.emit('enqueue', {
        x: from[0],
        y: from[1],
        z: 0.0,
      });

      this.sioClient.emit('enqueue', {
        x: to[0],
        y: to[1],
        z: 0.0,
      });

      return `Pedido realizado com sucesso! O número do seu pedido é: \n - ${order.code} \n Assim que chegarmos na sua localização você será informado! 😀`;
    } catch (e) {
      console.log(e);
      return 'Não foi possível realizar o seu pedido, por favor contate um administrador.';
    }
  }
}

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
