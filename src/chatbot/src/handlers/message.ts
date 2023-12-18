import { PrismaClient, User as PrismaUser } from '@prisma/client';
import { io } from 'socket.io-client';
import UserService from '../models/user';
import { Client, Message, List } from 'whatsapp-web.js';
import {
  handleCancelOrder,
  handleCreateUser,
  handleLeadAcess,
  handleNewOrder,
  handleProcessRequest,
  handleRequestMenu,
  handleStatusOrder,
  handleUpdateName,
  handleUpdateUser,
  handleUpdateUserSpeedVoice,
  handleUpdateUserVoice,
} from '../messages/user/user-messages';

import {
  handleAdminProcessRequest,
  handleAdminRequestMenu,
  handleNewPoint,
  handleUpdateUserAccess,
} from '../messages/admin/admin-messages';

// Define interfaces for the Command and Chain of Responsibility patterns
interface IRequestUserHandler {
  handle(
    requestState: number,
    message: Message,
    userName: string
  ): Promise<void>;
}

interface IRequestLeadHandler {
  handle(message: Message, user: PrismaUser | null): Promise<void>;
}

interface IMessageValidator {
  validate(message: Message): Promise<boolean>;
}

class RequestAdminHandler implements IRequestUserHandler {
  private whatsappClient: Client;
  private userService: UserService;

  constructor(whatsappClient: Client, userService: UserService) {
    this.whatsappClient = whatsappClient;
    this.userService = userService;
  }

  async handle(requestState: number, message: Message): Promise<void> {
    switch (requestState) {
      case 1:
        handleAdminRequestMenu(message, this.whatsappClient);
        break;
      case 2:
        handleAdminProcessRequest(message, this.whatsappClient);
        break;
      case 3:
        handleNewPoint(message, this.whatsappClient);
      case 4:
        handleUpdateUserAccess(message, this.whatsappClient);
      default:
        break;
    }
  }
}

// Command Pattern for handling different request states
class RequestUserHandler implements IRequestUserHandler {
  private whatsappClient: Client;
  private userService: UserService;

  constructor(whatsappClient: Client, userService: UserService) {
    this.whatsappClient = whatsappClient;
    this.userService = userService;
  }

  async handle(requestState: number, message: Message): Promise<void> {
    switch (requestState) {
      case 1:
        handleRequestMenu(message, this.whatsappClient);
        break;
      case 2:
        handleProcessRequest(message, this.whatsappClient);
        break;
      case 3:
        handleNewOrder(message, this.whatsappClient);
        break;
      case 4:
        handleStatusOrder(message, this.whatsappClient);
        break;
      case 5:
        handleCancelOrder(message, this.whatsappClient);
        break;
      case 6:
        handleUpdateName(message, this.whatsappClient);
        break;
      default:
        break;
    }
  }
}

class RequestLeadHandler implements IRequestLeadHandler {
  private whatsappClient: Client;
  private userService: UserService;

  constructor(whatsappClient: Client, userService: UserService) {
    this.whatsappClient = whatsappClient;
    this.userService = userService;
  }

  async handle(message: Message, user: PrismaUser | null): Promise<void> {
    if (user == null) {
      handleCreateUser(message, this.whatsappClient);
    }
    if (user?.name == '' && user != null) {
      handleUpdateUser(message, this.whatsappClient);
    }
    if (
      user?.name != '' &&
      user != null &&
      user?.voice != '' &&
      user?.speedVoice != 0.0
    ) {
      handleLeadAcess(message, this.whatsappClient);
    }
    if (
      user?.name != '' &&
      user != null &&
      user?.voice == '' &&
      user?.speedVoice == 0.0
    ) {
      handleUpdateUserVoice(message, this.whatsappClient);
    }
    if (
      user?.name != '' &&
      user != null &&
      user?.voice != '' &&
      user?.speedVoice == 0.0
    ) {
      handleUpdateUserSpeedVoice(message, this.whatsappClient);
    }
  }
}

// Base class for Chain of Responsibility Pattern
abstract class MessageValidator implements IMessageValidator {
  protected nextValidator?: IMessageValidator;

  constructor(nextValidator?: IMessageValidator) {
    this.nextValidator = nextValidator;
  }

  async validate(message: Message): Promise<boolean> {
    if (this.nextValidator) {
      return this.nextValidator.validate(message);
    }
    return true; // If all validations pass
  }
}

// Specific validator for group messages
class GroupMessageValidator extends MessageValidator {
  async validate(message: Message): Promise<boolean> {
    if ((await message.getChat()).isGroup) {
      return false; // Group messages are ignored
    }
    return super.validate(message);
  }
}

// Specific validator for checking bot readiness
class BotReadyValidator extends MessageValidator {
  validate(message: Message): Promise<boolean> {
    if (
      botReadyTimestamp == null ||
      new Date(message.timestamp * 1000) < botReadyTimestamp
    ) {
      return Promise.resolve(false); // Bot not ready or old message
    }
    return super.validate(message);
  }
}

// Your MessageEventHandler class
export class MessageEventHandler {
  private prismaClient: PrismaClient;
  private userService: UserService;
  private whatsappClient: Client;
  private messageValidator: IMessageValidator;

  constructor(
    prismaClient: PrismaClient,
    userService: UserService,
    whatsappClient: Client
  ) {
    this.userService = userService;
    this.whatsappClient = whatsappClient;
    this.messageValidator = new BotReadyValidator(new GroupMessageValidator());
  }

  async handleIncomingMessage(message: Message): Promise<void> {
    if (!(await this.messageValidator.validate(message))) {
      return;
    }
    if (message.body.toLowerCase() == '!sair') {
      await this.whatsappClient.sendMessage(message.from, 'Até mais!');
      this.userService.updateRequestUser(message.from, 1);
      return;
    }
    const userData = await this.userService.getUser(message.from);

    if (userData?.role?.includes('USER')) {
      let requestState = userData?.requestState;
      const requestUserHandler = new RequestUserHandler(
        this.whatsappClient,
        this.userService
      );
      requestUserHandler.handle(requestState, message);
    }
    if (userData?.role?.includes('ADMIN')) {
      if (message.body.toLowerCase() == '!parar') {
        const socket = io(process.env.SOCKET_URL || '');
        socket.emit('enqueue', { x, y, z });
      }
      let requestState = userData?.requestState;
      const requestAdminHandler = new RequestAdminHandler(
        this.whatsappClient,
        this.userService
      );
      requestAdminHandler.handle(requestState, message);
    }
    if (userData?.role?.includes('LEAD') || userData == null) {
      let requestLeadHandler = new RequestLeadHandler(
        this.whatsappClient,
        this.userService
      );
      requestLeadHandler.handle(message, userData);
    }
  }
}

let botReadyTimestamp: Date | null = new Date();
