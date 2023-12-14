import { Inject, Injectable } from '@nestjs/common';
import { Chat, Message } from 'whatsapp-web.js';
import { UserService } from 'src/prisma/user.service';
import { AIService } from '../AI/AI.service';
import transformConversation from './utils/transformConversation';
import { UserDoesntExists } from 'src/prisma/user.service';
import { io, Socket } from 'socket.io-client';
import { ToolService } from '../prisma/tool.service';
import { LocationService } from '../prisma/location.service';
import { OrderService } from '../prisma/order.service';
import parseCoordinates from './utils/parseCoordinates';
import { HandleUserService } from './handle-user.service';
import { HandleLeadService } from './handle-lead.service';
import { HandleAdminService } from './handle-admin.service';

@Injectable()
export class HandlerService {
  private readonly sioClient: Socket;
  private readonly functionMapping;
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(AIService) private aiService: AIService,
    @Inject(ToolService) private toolService: ToolService,
    @Inject(LocationService) private locationService: LocationService,
    @Inject(OrderService) private orderService: OrderService,
    @Inject(HandleUserService) private handleUserService: HandleUserService,
    @Inject(HandleAdminService) private handleAdminService: HandleAdminService,
    @Inject(HandleLeadService) private handleLeadService: HandleLeadService,
  ) {
    this.sioClient = io(process.env.SOCKET_URL || '');

    this.functionMapping = {
      USER: this.handleUserService,
      ADMIN: this.handleAdminService,
      LEAD: this.handleLeadService,
    };
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

    const UserRole = (userData?.role as 'USER' | 'ADMIN' | 'LEAD') || 'LEAD';

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
      UserRole,
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
          return await this.functionMapping[UserRole][res.function](
            message.from,
            res.arguments && res.arguments,
          );
          // return await this[res.function](
          //   message.from,
          //   res.arguments && res.arguments,
          // );
        } catch (e) {
          return `${e}`;
        }
    }
  }
}
