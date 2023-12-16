import qrcode from 'qrcode';
import { Client, Events, LocalAuth } from 'whatsapp-web.js';
import process from 'process';
import constants from './constants';
import * as terminal from './cli/ui';
import { MessageEventHandler } from './handlers/message';
import { initOpenAI } from './providers/openai';
import express, { Request, Response } from 'express';
import { PrismaClient } from '@prisma/client';
import UserService from './models/user';
import dotenv from 'dotenv';

const app = express();
const port = 3000;

app.use((req, res, next) => {
  res.header('Access-Control-Allow-Origin', '*'); // Allow requests from any origin
  res.header(
    'Access-Control-Allow-Headers',
    'Origin, X-Requested-With, Content-Type, Accept'
  );
  next();
});

// Ready timestamp of the bot
let botReadyTimestamp: Date | null = null;

// Prisma

const prisma = new PrismaClient();
const userService = new UserService(prisma);
const AUTH_TOKEN = process.env.AUTH_TOKEN || '';
const TOKEN_SECRET = process.env.TOKEN_SECRET || '';

dotenv.config();

// Entrypoint
const start = async () => {
  if (btoa(AUTH_TOKEN) == btoa(TOKEN_SECRET)) {
    terminal.printIntro();

    // WhatsApp Client
    const client = new Client({
      puppeteer: {
        args: ['--no-sandbox'],
      },
      authStrategy: new LocalAuth({
        dataPath: constants.sessionPath,
      }),
    });

    let messageQueue: any = [];
    let qrCodeUrl: string | null = null;
    // WhatsApp auth
    client.on(Events.QR_RECEIVED, (qr: string) => {
      qrcode.toString(
        qr,
        {
          type: 'svg',
          margin: 2,
          scale: 1,
        },
        (err, url) => {
          if (err) throw err;
          terminal.print(url);
          qrCodeUrl = url;
        }
      );
    });

    app.get('/qrcode', (req: Request, res: Response) => {
      if (qrCodeUrl) {
        res.status(200).json({ success: true, qrcodeUrl: qrCodeUrl });
      } else {
        res
          .status(202)
          .json({ success: false, message: 'User is authenticated' });
      }
    });

    app.use(express.static(__dirname + '/public'));
    app.use('/js', express.static(__dirname + 'public/assets/js'));
    app.use('/css', express.static(__dirname + 'public/assets/css'));

    app.listen(port, () => {
      console.log(`Server running on http://localhost:${port}`);
    });

    // WhatsApp loading
    client.on(Events.LOADING_SCREEN, (percent) => {
      if (percent == '0') {
        terminal.printLoading();
      }
    });

    // WhatsApp authenticated
    client.on(Events.AUTHENTICATED, () => {
      qrCodeUrl = null;
      terminal.printAuthenticated();
    });

    // WhatsApp authentication failure
    client.on(Events.AUTHENTICATION_FAILURE, () => {
      terminal.printAuthenticationFailure();
    });

    // WhatsApp ready
    client.on(Events.READY, () => {
      // Set bot ready timestamp
      botReadyTimestamp = new Date();
      initOpenAI();
    });

    const messageEventHandler = new MessageEventHandler(
      prisma,
      userService,
      client
    );

    // WhatsApp message
    client.on(Events.MESSAGE_RECEIVED, async (message: any) => {
      // Add message to queue
      messageQueue.push(message);
      // Handle messages in queue
      while (messageQueue.length > 0) {
        // Get message from queue
        const message: any = messageQueue.shift();
        // Ignore if message is from status broadcast
        if (message.from == constants.statusBroadcast) return;

        // Ignore if it's a quoted message, (e.g. Bot reply)
        if (message.hasQuotedMsg) return;

        messageEventHandler.handleIncomingMessage(message);
      }
    });

    // WhatsApp initialization
    client.initialize();
  }
};

start();

export { botReadyTimestamp };