import { Inject, Injectable } from '@nestjs/common';
import { Client, Events, LocalAuth } from 'whatsapp-web.js';
import constants from './constants';
import qrcode from 'qrcode';
import { HandlerService } from './handler.service';

@Injectable()
export class WhatsappService {
  private client: Client;
  private qrCodeUrl: string | null = null;
  public botReadyTimestamp: Date | null = null;
  private messageQueue: any = [];

  constructor(@Inject(HandlerService) private handlerService: HandlerService) {
    if (btoa(process.env.AUTH_TOKEN) != btoa(process.env.TOKEN_SECRET))
      throw new Error('Token inválido para o chatbot');
    console.log('starting chatbot...');
    this.client = new Client({
      puppeteer: {
        args: ['--no-sandbox'],
      },
      authStrategy: new LocalAuth({
        dataPath: constants.sessionPath,
      }),
    });
    this.initializeClient();
  }

  initializeClient() {
    this.client.on(Events.QR_RECEIVED, (qr: string) => {
      qrcode.toString(
        qr,
        {
          type: 'svg',
          margin: 2,
          scale: 1,
        },
        (err, url) => {
          if (err) throw err;
          console.log(url);
          this.qrCodeUrl = url;
        },
      );
    });

    this.client.on(Events.LOADING_SCREEN, (percent) => {
      if (percent == '0') {
        console.log('loading');
      }
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
      this.botReadyTimestamp = new Date();
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

        this.handlerService.handleIncomingMessage(message);
      }
    });
  }

  getQrCodeUrl(): string | null {
    return this.qrCodeUrl;
  }

  sendMessage(to: string, message: string): void {
    this.client.sendMessage(to, message);
  }
}
