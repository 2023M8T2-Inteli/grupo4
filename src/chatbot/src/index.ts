import qrcode from "qrcode";
import { Client, Message, Events, List, LocalAuth } from "whatsapp-web.js";
import process from "process";
// Constants
import constants from "./constants";

// CLI
import * as cli from "./cli/ui";
// import { handleIncomingMessage } from "./handlers/message";
import {MessageEventHandler} from "./handlers/message";


// Ready timestamp of the bot
let botReadyTimestamp: Date | null = null;

// Prisma
import { PrismaClient } from "@prisma/client";
const prisma = new PrismaClient();

// User Service
import UserService from "./models/user";
const userService = new UserService(prisma);


import dotenv from "dotenv";
dotenv.config();

const AUTH_TOKEN = process.env.AUTH_TOKEN || "";
const TOKEN_SECRET = process.env.TOKEN_SECRET || "";

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

	// WhatsApp auth
	client.on(Events.QR_RECEIVED, (qr: string) => {
		console.log("");
		qrcode.toString(
			qr,
			{
				type: "terminal",
				small: true,
				margin: 2,
				scale: 1
			},
			(err, url) => {
				if (err) throw err;
				cli.printQRCode(url);
			}
		);
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
