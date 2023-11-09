import { PrismaClient } from "@prisma/client";
import UserService from "../models/user";
import { Client, Message, List } from "whatsapp-web.js";
import { botReadyTimestamp } from "../index";
import * as cli from "../cli/ui";
// Config & Constants
import config from "../config";
import { handleCreateUser, handleRequestMenu, handleRequestUser, updateRequest } from "../services/user/user-service";

export default class MessageEventHandler {
	private userService: UserService;
	private prisma: PrismaClient;
	private whatsappClient: Client;

	constructor(prisma: PrismaClient, userService: UserService, whatsappClient: Client) {
		this.prisma = prisma;
		this.userService = userService;
		this.whatsappClient = whatsappClient;
	}

	async handleRequestState(requestState: number, message: Message, userName: string) {
		switch (requestState) {
			case 0:
				handleCreateUser(message, this.whatsappClient, userName);
				break;
			case 1:
				// call handleRequestUser
				handleRequestUser(message, this.whatsappClient);
				break;
			case 2:
				handleRequestMenu(message, this.whatsappClient);
				break;
			case 3:
				
			default:
				break;
		}
	}

	async handleIncomingMessage(message: Message, userName: string) {
		// Prevent handling old messages
		if (message.timestamp != null) {
			const messageTimestamp = new Date(message.timestamp * 1000);

			// If startTimestamp is null, the bot is not ready yet
			if (botReadyTimestamp == null) {
				cli.print(`Ignoring message because bot is not ready yet: ${message.body}`);
				return;
			}

			// Ignore messages that are sent before the bot is started
			if (messageTimestamp < botReadyTimestamp) {
				cli.print(`Ignoring old message: ${message.body}`);
				return;
			}
		}

		// ignore message from groups if groupchatsEnabled is false
		if ((await message.getChat()).isGroup && !config.groupchatsEnabled) return;

		let requestState = 0;
		const userData = await this.userService.getUser(message.from);

		if (userData) {
			requestState = userData.requestState;
		}

		this.handleRequestState(requestState, message, userName);
	}
}
