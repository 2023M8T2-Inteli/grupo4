import { Inject, Injectable } from '@nestjs/common';
import { Chat, Message } from 'whatsapp-web.js';
import { UserService } from 'src/prisma/user.service';
import { AIService } from '../AI/AI.service';
import transformConversation from './utils/transformConversation';
import { UserDoesntExists } from 'src/prisma/user.service';
import { io, Socket } from 'socket.io-client';
import { ToolService } from '../prisma/tool.service';
import { LocationService } from '../prisma/location.service';
import { Role } from '@prisma/client';
import { OrderService } from '../prisma/order.service';
import parseCoordinates from './utils/parseCoordinates';

interface CreateUserArgs {
  firstName: string;
  lastName: string;
  document: string;
}

interface CreateNewOrderArgs {
  from: number[];
  to: number[];
}

@Injectable()
export class HandlerService {
  private readonly sioClient: Socket;
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(AIService) private aiService: AIService,
    @Inject(ToolService) private toolService: ToolService,
    @Inject(LocationService) private locationService: LocationService,
    @Inject(OrderService) private orderService: OrderService,
  ) {
    this.sioClient = io(process.env.SOCKET_URL || '');
  }

  async handleIncomingMessage(message: Message): Promise<any> {
    let userData;
    try {
      userData = await this.userService.getUser(message.from);
    } catch (e) {
      if (e instanceof UserDoesntExists) {
        userData = null;
      } else {
        return 'Um erro aconteceu, contate um administrador.';
      }
    }

    const chat: Chat = await message.getChat();

    const messages: Message[] = await chat.fetchMessages({ limit: 20 });

    const parsedMessages = transformConversation(messages);
    const toolCoordinates = await this.toolService.getAllTools();
    const locationCoordinates = await this.locationService.getAllLocations();
    const { parsedLocations, parsedTools } = parseCoordinates(
      toolCoordinates,
      locationCoordinates,
    );

    const res = await this.aiService.callGPT(
      (userData?.role as 'USER' | 'ADMIN' | 'LEAD') || 'LEAD',
      parsedTools,
      parsedLocations,
      parsedMessages,
    );

    switch (res.type) {
      case 'message':
        return res.message;
      case 'function_call':
        try {
          console.log(res.function);
          console.log(
            this[res.function](
              message.from,
              res.arguments ? res.arguments : {},
            ),
          );
          return await this[res.function](
            message.from,
            res.arguments && res.arguments,
          );
        } catch (e) {
          return `${e}`;
        }
    }
  }

  async handleCreateUser(userPhone: string, args: CreateUserArgs) {
    const firstName = args?.firstName || '';
    const lastName = args?.lastName || '';
    const document = args?.document || '';

    if (document && firstName && lastName)
      return 'Sua conta foi criada! 😀 Você deve aguardar que um administrador libere seu acesso!';

    return `Preciso de mais algumas informações! Me envie: ${
      firstName ? '' : '\n - primeiro nome,'
    }
    ${lastName ? '' : '\n -sobrenome,'}
    ${document ? '' : '\n -documento'}. \n  😀`;
  }

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  async handleLeadAccess(userPhone: string, _args: object) {
    try {
      const user = await this.userService.getUser(userPhone);

      if (user.role != 'LEAD') {
        return 'Parece que você acabou de ganhar um up no nosso sistema! Em que posso lhe ajudar?';
      } else {
        return 'Opa! Encontrei o seu cadastro aqui, mas você ainda não está com permissões de acessar nosso serviço, em breve um administrador irá lhe contatar.';
      }
    } catch (e) {
      if (e instanceof UserDoesntExists) {
        return 'Parece que você ainda não esta cadastrado em nosso sistema. Você gostaria de fazer o cadastro agora?';
      } else {
        console.log(`Error: ${e}`);
        return 'Não foi possível consultar a sua situação, por favor consulte um administrador.';
      }
    }
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

  private async generateNewOrder(
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
