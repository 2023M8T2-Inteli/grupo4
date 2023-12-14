import { Inject, Injectable } from '@nestjs/common';
import { Chat, Message } from 'whatsapp-web.js';
import { UserService } from 'src/prisma/user.service';
import { AIService } from '../AI/AI.service';
import transformConversation from './utils/transformConversation';
import { UserDoesntExists } from 'src/prisma/user.service';
import { PointService } from 'src/prisma/coordinates.service';
import parseCoordinates from './utils/parseCoordinates';

interface CreateUserArgs {
  firstName: string;
  lastName: string;
  document: string;
}

@Injectable()
export class HandlerService {
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(AIService) private aiService: AIService,
    @Inject(PointService) private pointService: PointService,
  ) {}

  async handleIncomingMessage(message: Message): Promise<any> {
    const userData = await this.userService.getUser(message.from);

    const chat: Chat = await message.getChat();

    const messages: Message[] = await chat.fetchMessages({ limit: 20 });

    const parsedMessages = transformConversation(messages);

    const toolCoordinates = await this.toolService.getAllCoordinates();
    const locationCoordinates = await this.locationService.getAllCoordinates();
    const {parsedToolCoordinates, parsedLocationCoordinates} = parseCoordinates(tools, locations);

    console.log(chat);

    const res = await this.aiService.callGPT(
      (userData?.role as 'USER' | 'ADMIN' | 'LEAD') || 'LEAD',
      parsedToolCoordinates,
      parsedLocationCoordinates,

    );
    console.log(res);

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

    // return res.type == 'message' ? res.message : null;
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
}
