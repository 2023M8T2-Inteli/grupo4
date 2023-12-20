import { forwardRef, Inject, Injectable } from '@nestjs/common';
import { io, Socket } from 'socket.io-client';
import { Order, Point, Role, Tool, User } from '@prisma/client';
import {
  NothingToUpdate,
  UserDoesntExists,
  UserService,
} from '../prisma/user.service';
import {
  OpenOrdersDoestExist,
  OrderDoesntExists,
  OrdersEmpty,
  OrderService,
} from '../prisma/order.service';
import { LocationService } from '../prisma/location.service';
import { ToolService } from '../prisma/tool.service';
import { AIService } from '../AI/AI.service';
import { WhatsappService } from '../whatsapp/whatsapp.service';
import { MessageMedia } from 'whatsapp-web.js';
import { TranscriptionService } from '../prisma/transcription.service';
import { WebsocketService } from 'src/websockets/websocket.service';

interface CreateNewOrderArgs {
  from: number[];
  to: number[];
}

@Injectable()
export class HandleUserService {
  protected readonly sioClient: Socket;
  protected readonly permissionMapping;

  constructor(
    @Inject(UserService) protected userService: UserService,
    @Inject(OrderService) protected orderService: OrderService,
    @Inject(LocationService) protected locationService: LocationService,
    @Inject(ToolService) protected toolService: ToolService,
    @Inject(AIService) protected aiService: AIService,
    @Inject(forwardRef(() => WhatsappService))
    protected whatsappService: WhatsappService,
    @Inject(TranscriptionService)
    protected transcriptionService: TranscriptionService,
    @Inject(WebsocketService) protected websocketService: WebsocketService,
  ) {
    this.sioClient = io(process.env.SOCKET_URL || '');
    this.permissionMapping = {
      USER: [Role.USER, Role.ADMIN],
      ADMIN: [Role.ADMIN],
      LEAD: [Role.LEAD, Role.USER, Role.ADMIN],
    };
  }

  async handleNewOrder(userPhone: string, args: CreateNewOrderArgs) {
    const permissionMessage = await this.checkPermission(userPhone, Role.USER);

    if (permissionMessage) return permissionMessage;

    const from = args?.from;
    const to = args?.to;

    if (from?.length === 2 && to?.length === 2) {
      return await this.generateNewOrder(userPhone, from, to);
    }

    return `Não foi possível processar o seu pedido. As seguintes informações estão faltando: ${
      from?.length !== 2 && '\n - origem do pedido'
    } ${to?.length !== 2 && '\n - destino do pedido'}. \n  😀`;
  }

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  async handleGetAllOrders(userPhone: string, _args: object) {
    const permissionMessage = await this.checkPermission(userPhone, Role.USER);
    if (permissionMessage) return permissionMessage;
    try {
      const orders = await this.orderService.getAllOrders(userPhone);

      return await this.formatOrders(orders);
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que houve um erro aqui no sistema e você ainda não tem um cadastro conosco. Gostaria de fazer um agora? 😀';
      if (e instanceof OrdersEmpty)
        return 'Você ainda não possui nenhum pedido. Gostaria de fazer um agora?';
      return 'Um erro aconteceu, contate um administrador.';
    }
  }

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  async handleGetAllOpenOrders(userPhone: string, _args: object) {
    const permissionMessage = await this.checkPermission(userPhone, Role.USER);
    if (permissionMessage) return permissionMessage;
    try {
      const orders = await this.orderService.getAllOpenOrders(userPhone);

      return await this.formatOrders(orders);
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que houve um erro aqui no sistema e você ainda não tem um cadastro conosco. Gostaria de fazer um agora? 😀';
      if (e instanceof OrdersEmpty)
        return 'Você ainda não possui nenhum pedido. Gostaria de fazer um agora?';
      if (e instanceof OpenOrdersDoestExist)
        return 'Não há pedidos abertos. Gostaria de ver todos os seus pedidos?';
      return 'Um erro aconteceu, contate um administrador.';
    }
  }

  async handleGetOrderStatus(userPhone: string, args: { orderId: number }) {
    const permissionMessage = await this.checkPermission(userPhone, Role.USER);
    if (permissionMessage) return permissionMessage;
    try {
      const order = await this.orderService.getOrderByCode(
        userPhone,
        args.orderId,
      );

      return await this.formatOrder(order);
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que houve um erro aqui no sistema e você ainda não tem um cadastro conosco. Gostaria de fazer um agora? 😀';
      if (e instanceof OrderDoesntExists)
        return 'Não consegui encontrar nenhuma ordem com esse código. Você gostaria de ver todos os seus pedidos?';
      return 'Um erro aconteceu, contate um administrador.';
    }
  }

  async handleCancelOpenOrder(userPhone: string, args: { orderId: number }) {
    const permissionMessage = await this.checkPermission(userPhone, Role.USER);
    if (permissionMessage) return permissionMessage;
    try {
      const order = await this.orderService.cancelOrder(
        userPhone,
        args.orderId,
      );

      return `Pedido ${order.code} cancelado com sucesso! \n Ficamos tristes em saber que você não quer mais o produto. 😢, gostaria de solicitar outro?`;
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que houve um erro aqui no sistema e você ainda não tem um cadastro conosco. Gostaria de fazer um agora? 😀';
      if (e instanceof OrderDoesntExists)
        return 'Não consegui encontrar nenhuma ordem com esse código. Você gostaria de ver todos os seus pedidos?';
      return 'Um erro aconteceu, contate um administrador.';
    }
  }

  async handleChangeUserInfo(userPhone: string, args: Partial<User>) {
    const permissionMessage = await this.checkPermission(userPhone, Role.USER);
    if (permissionMessage) return permissionMessage;
    try {
      const updatedUser = await this.userService.updateUserData({
        ...args,
        cellPhone: userPhone,
      });

      if (updatedUser) return 'Dados atualizados com sucesso! 😀';
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que houve um erro aqui no sistema e você ainda não tem um cadastro conosco. Gostaria de fazer um agora? 😀';

      if (e instanceof NothingToUpdate)
        return 'Ops, parece que houve um problema ao atualizar seus dados. 😢. \n Você consegue refazer a solicitação?';

      return 'Um erro aconteceu, contate um administrador.';
    }
  }

  async handleSendAudio(userPhone: string, args: { res: string }) {
    const { res } = args;
    if (!res)
      return 'Não foi possível responder nesse momento. Gostaria de fazer um novo pedido?';

    const permissionMessage = await this.checkPermission(userPhone, Role.USER);

    if (permissionMessage) return permissionMessage;

    const audioBase64 = await this.aiService.text2Speech(res);

    const messageMedia = new MessageMedia('audio/mp3', audioBase64);

    const sendedMessage = await this.whatsappService.sendMessage(
      userPhone,
      messageMedia,
      {
        sendAudioAsVoice: true,
      },
    );

    if (sendedMessage) {
      await this.transcriptionService.insertNewTranscription(
        sendedMessage.id.id,
        sendedMessage.mediaKey,
        res,
      );
    }

    return;
  }

  async handleConfirmOrder(
    userPhone: string,
    args: { orderId: number; delivered: boolean },
  ) {
    try {
      const permissionMessage = await this.checkPermission(
        userPhone,
        Role.USER,
      );

      if (permissionMessage) return permissionMessage;

      const { orderId, delivered } = args;

      if (
        (!Number(orderId) && Number(orderId) !== 0) ||
        typeof delivered !== 'boolean'
      )
        return 'Não foi possível confirmar o pedido. Precisa de mais alguma outra ajuda?';

      if (delivered) {
        const updatedOrder = await this.orderService.updateOrderStatusByCode(
          orderId,
          'Finished',
        );

        this.websocketService.emergencyStop({ emergency_stop: 0 });

        return `Pedido ${updatedOrder.code} confirmado com sucesso! \n Obrigado por utilizar nossos serviços, esperamos que tenha gostado! 😁`;
      }

      const updatedOrder = await this.orderService.updateOrderStatusByCode(
        orderId,
        'Canceled',
      );

      this.whatsappService.sendAdminContact(userPhone);

      this.websocketService.emergencyStop({ emergency_stop: 0 });

      return `Ficamos muito tristes em saber que não conseguimos entregar o seu pedido. 😢. \n O pedido ${updatedOrder.code} foi cancelado com sucesso.\n Estou mandando aqui um contato de um administrador, caso queira reportar algum problema. \n Gostaria de fazer um novo pedido?`;
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que houve um erro aqui no sistema e você ainda não tem um cadastro conosco. Gostaria de fazer um agora? 😀';
      if (e instanceof OrderDoesntExists)
        return 'Não consegui encontrar nenhuma ordem com esse código. Você gostaria de ver todos os seus pedidos?';
      return 'Um erro aconteceu, contate um administrador.';
    }
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

      console.log(
        `[handleUserService] 📦 \x1b[35m Pedido \x1b[31m GRAB \x1b[35m enviado! \x1b[0m`,
      );

      this.websocketService.addPointToQueue({
        id: order.code.toString(),
        type: 'GRAB',
        x: from[0],
        y: from[1],
      });

      await this.timeOut(500);

      console.log(
        `[handleUserService] 🔚 \x1b[35m Pedido \x1b[31m DROP \x1b[35m enviado! \x1b[0m`,
      );

      this.websocketService.addPointToQueue({
        id: order.code.toString(),
        type: 'DROP',
        x: to[0],
        y: to[1],
      });

      return `Pedido realizado com sucesso! O número do seu pedido é: \n - ${order.code} \n Assim que chegarmos na sua localização você será informado! 😀`;
    } catch (e) {
      console.log(e);
      return 'Não foi possível realizar o seu pedido, por favor contate um administrador.';
    }
  }

  protected timeOut(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  protected async formatOrders(orders: Order[]) {
    let message = 'Encontrei aqui no meu sistema os seguintes pedidos:';

    for (const order of orders) {
      try {
        const tool: Tool = await this.toolService.getToolById(order.toolId);
        const location: Point = await this.locationService.getLocationById(
          order.pointId,
        );
        message += `
      \n 📦 *Código do pedido*: ${order.code}
      \n - Ferramenta pedida: ${tool.name}
      \n - Destino de entrega: ${location.name}
      \n - Status: ${order.type}
      `;
      } catch (e) {
        console.log(e);
      }
    }

    return message;
  }

  protected async formatOrder(order: Order) {
    try {
      const tool: Tool = await this.toolService.getToolById(order.toolId);
      const location: Point = await this.locationService.getLocationById(
        order.pointId,
      );
      return `
      \n 📦 *Código do pedido*: ${order.code}
      \n - Ferramenta pedida: ${tool.name}
      \n - Destino de entrega: ${location.name}
      \n - Status: ${order.type}
      `;
    } catch (e) {
      console.log(e);
    }
  }

  protected async checkPermission(userPhone: string, permissionLevel: Role) {
    try {
      const userRole = await this.userService.getUserRole(userPhone);
      console.log(
        `User role: ${userRole} e permissions: ${this.permissionMapping[permissionLevel]}`,
      );
      if (!this.permissionMapping[permissionLevel].includes(userRole))
        return 'Ops, parece que houve um erro no sistema e você não tem permissão para essa ação 🥲. Gostaria de fazer outra solicitação?';
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que você ainda não tem um cadastro conosco. Você gostaria de se cadastrar?';
    }
    return null;
  }
}
