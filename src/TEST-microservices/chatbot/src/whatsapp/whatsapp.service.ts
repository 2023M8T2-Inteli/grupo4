import { forwardRef, Inject, Injectable } from '@nestjs/common';
import { Client, Events, LocalAuth } from 'whatsapp-web.js';
import constants from './constants';
import qrcode from 'qrcode-terminal';
import { HandlerService } from '../handler/handler.service';
import { check_out, validate_message } from '../handler/utils/validate_msg';

@Injectable()
export class WhatsappService {
  private client: Client;
  private qrCodeUrl: string | null = null;
  public botReadyTimestamp: number | null = null
  private messageQueue: any = [];

  constructor(
    @Inject(HandlerService)
    private handlerService: HandlerService,
  ) {
    if (btoa(process.env.AUTH_TOKEN) != btoa(process.env.TOKEN_SECRET))
      throw new Error('Token inválido para o chatbot');
    console.log('starting chatbot...');

    this.client = new Client({
      authStrategy: new LocalAuth({
        dataPath: './',
      }),
    });
    console.log("wpp client created");
    this.initializeClient();
  }

  initializeClient() {
    console.log('client initializing');

    this.client.on('qr', (qr: string) => {
      console.log('NEW QR -- ' + qr);
      qrcode.generate(qr, { small: true });
    });

    this.client.on(Events.LOADING_SCREEN, (percent) => {
        console.log(`loading... ${percent}%`);
    });

    // WhatsApp authenticated
    this.client.on(Events.AUTHENTICATED, () => {
      console.log('authenticated to whatsapp!');
    });

    // WhatsApp authentication failure
    this.client.on(Events.AUTHENTICATION_FAILURE, () => {
      console.log('authentication failure');
    });

    // WhatsApp ready
    this.client.on(Events.READY, () => {
      this.botReadyTimestamp = Math.floor(+new Date()/1000);
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

  getQrCodeUrl(): string | null {
    return this.qrCodeUrl;
  }

  sendMessage(to: string, message: string): void {
    this.client.sendMessage(to, message);
  }
}
