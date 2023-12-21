import { forwardRef, Inject, Injectable } from '@nestjs/common';
import { Client, Events, LocalAuth, Message } from 'whatsapp-web.js';
import constants from './constants';
import { HandlerService } from '../handler/handler.service';
import { check_out, validate_message } from '../handler/utils/validate_msg';
import { UserService } from 'src/prisma/user.service';
import * as qrcode from 'qrcode';

@Injectable()
export class WhatsappService {
  private client: Client;
  private qrCodeUrl: string | null = null;
  public botReadyTimestamp: number | null = null;
  private messageQueue: any = [];
  private isAuthenticated: boolean;

  constructor(
    @Inject(HandlerService)
    private handlerService: HandlerService,
    @Inject(UserService) private userService: UserService,
  ) {
    if (btoa(process.env.AUTH_TOKEN) != btoa(process.env.TOKEN_SECRET))
      throw new Error('Token inválido para o chatbot');
    console.log('[whatsappService] iniciando chatbot...');

    this.client = new Client({
      authStrategy: new LocalAuth({
        dataPath: './',
      }),
    });
    console.log('[whatsappService] wpp client criado');
    this.initializeClient();

    this.isAuthenticated = false;
  }

  initializeClient() {
    console.log('[whatsappService] inicializando...');

    this.client.on('qr', (qr: string) => {
      this.qrCodeUrl = qr;
    });

    this.client.on(Events.LOADING_SCREEN, (percent) => {
      console.log(`[whatsappService] loading... ${percent}%`);
    });

    // WhatsApp authenticated
    this.client.on(Events.AUTHENTICATED, () => {
      this.isAuthenticated = true;
      console.log(
        '[whatsappService]\x1b[36m\x1b[1m Authenticated to whatsapp!\x1b[0m',
      );
    });

    // WhatsApp authentication failure
    this.client.on(Events.AUTHENTICATION_FAILURE, () => {
      this.isAuthenticated = false;
      console.log('authentication failure');
    });

    // WhatsApp ready
    this.client.on(Events.READY, () => {
      this.botReadyTimestamp = Math.floor(+new Date() / 1000);
    });

    // WhatsApp message
    this.client.on(Events.MESSAGE_RECEIVED, async (message: any) => {
      // Add message to queue
      this.messageQueue.push(message);
      // Handle messages in queue
      while (this.messageQueue.length > 0) {
        // Get message from queue
        const message: any = this.messageQueue.shift();
        // Ignore if message is from status broadcast
        if (message.from == constants.statusBroadcast) return;

        // Ignore if it's a quoted message, (e.g. Bot reply)
        if (message.hasQuotedMsg) return;

        if (!(await validate_message(message, this.botReadyTimestamp))) {
          return;
        }
        // checka se a mensagem enviada é "!sair"
        if (await check_out(message, this)) {
          return;
        }

        const msg = await this.handlerService.handleIncomingMessage(message);
        if (msg) message.reply(msg);
      }
    });

    this.client.initialize();
  }

  getQrCodeUrl(): any {
    if (this.isAuthenticated) return {qr: "", isAuthenticated: true}
    if (!this.qrCodeUrl) return  {qr: "", isAuthenticated: false}
    return {qr: this.qrCodeUrl, isAuthenticated: false};
  }

  async sendMessage(to: string, message: any, options = {}): Promise<Message> {
    return await this.client.sendMessage(to, message, options);
  }

  async getContactFromID(id: string) {
    return await this.client.getContactById(id);
  }
  async sendAdminContact(userPhone: string) {
    const admin = await this.userService.getAdmin();
    const adminContact = await this.getContactFromID(admin.cellPhone);
    this.sendMessage(userPhone, adminContact);
  }
}
