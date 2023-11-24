import qrcode from "qrcode";
import { Client, Events, LocalAuth } from "whatsapp-web.js";
import process from "process";
import constants from "./constants";
import * as cli from "./cli/ui";
import { initAiConfig } from "./handlers/ai-config";
import {MessageEventHandler} from "./handlers/message";
import { initOpenAI } from "./providers/openai";
import express, { Request, Response } from 'express';
import { PrismaClient } from "@prisma/client";
import UserService from "./models/user";
import dotenv from "dotenv";
import { io } from "socket.io-client";

const app = express();
const port = 3000;

// Ready timestamp of the bot
let botReadyTimestamp: Date | null = null;

// Prisma

const prisma = new PrismaClient();
const userService = new UserService(prisma);
const AUTH_TOKEN = process.env.AUTH_TOKEN || "";
const TOKEN_SECRET = process.env.TOKEN_SECRET || "";



dotenv.config();

// Entrypoint
const start = async () => {
	if(btoa(AUTH_TOKEN) == btoa(TOKEN_SECRET)){
	cli.printIntro();

	// WhatsApp Client
	const client = new Client({
		puppeteer: {
			args: ["--no-sandbox"]
		},
		authStrategy: new LocalAuth({
			dataPath: constants.sessionPath
		})
	});

	let messageQueue: any = [];
	let qrCodeUrl: string | null = null;
	// WhatsApp auth
	client.on(Events.QR_RECEIVED, (qr: string) => {
		let qr_code = qrcode.toString(
			qr,
			{
				type: "svg",
				width: 300,
				margin: 2,
				scale: 1
			},
			(err, url) => {
				if (err) throw err;
				cli.printQRCode(url);
				qrCodeUrl = url;
			}
		);	
	});
	
	
	app.get('/', (req: Request, res: Response) => {
		if (qrCodeUrl) {
			res.status(200).send(qrCodeUrl);
		} else {
			res.status(400).send('User is authenticated');
		}
	});
	
	app.listen(port, () => {
		console.log(`Server running on http://localhost:${port}`);
	});

	// WhatsApp loading
	client.on(Events.LOADING_SCREEN, (percent) => {
		if (percent == "0") {
			cli.printLoading();
		}
	});

	// WhatsApp authenticated
	client.on(Events.AUTHENTICATED, () => {
		cli.printAuthenticated();
	});

	// WhatsApp authentication failure
	client.on(Events.AUTHENTICATION_FAILURE, () => {
		cli.printAuthenticationFailure();
	});

	// WhatsApp ready
	client.on(Events.READY, () => {
		// Set bot ready timestamp
		botReadyTimestamp = new Date();
		initAiConfig();
		initOpenAI();
	
	});

	const messageEventHandler = new MessageEventHandler(prisma, userService, client);

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
}};

start();

export { botReadyTimestamp };
