import { PrismaClient } from "@prisma/client";
import UserService from "../models/user";
import { Client, Message, List } from "whatsapp-web.js";
import { botReadyTimestamp } from "../index";
import * as cli from "../cli/ui";
// Config & Constants
import config from "../config";
import { delay, handleCreateUser, handleRequestMenu, handleRequestNewPiece, handleRequestUser, sendContact, updateRequest } from "../services/user/user-service";

import { handleMessageAIConfig, getConfig, executeCommand } from "../handlers/ai-config";

// // Speech API & Whisper
import { TranscriptionMode } from "../types/transcription-mode";
import { transcribeRequest } from "../providers/speech";
import { transcribeAudioLocal } from "../providers/whisper-local";
import { transcribeWhisperApi } from "../providers/whisper-api";
import { transcribeOpenAI } from "../providers/openai";
import LeadService from "../models/lead";

const { v4: uuidv4 } = require("uuid");

export default class MessageEventHandler {
	private userService: UserService;
	private prisma: PrismaClient;
	private whatsappClient: Client;
	private leadService: LeadService;

	constructor(prisma: PrismaClient, userService: UserService, leadService: LeadService, whatsappClient: Client) {
		this.prisma = prisma;
		this.userService = userService;
		this.leadService = leadService;
		this.whatsappClient = whatsappClient;
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

		const userData = await this.userService.getUser(message.from);

		if (userData) {
			let requestState = userData?.requestState || 0;
			this.handleRequestState(requestState, message, userName);
		}else{
			this.handleRequestAccess(message, userName);
		}
	}

	async handleRequestAccess(message: Message, userName: string) {
		const newLead = {
			id: uuidv4(),
			name: userName,
			cellPhone: message.from,
			createdAt: new Date()
		};

		this.leadService.createAccountLead(newLead);

		message.reply(`Olá ${userName}, tudo bem?`);

		await delay(1000);

		message.reply("Estou verificando se você tem autorização de acesso ao sistema, por favor, aguarde um momento.");

		await delay(1000);
		
		message.reply("Identifiquei aqui que você não possui autorização de acesso ao sistema, por favor, entre em contato com um administrador.");

		
		await delay(1000);

		sendContact(message, this.whatsappClient);

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
				handleRequestNewPiece(message, this.whatsappClient);
				// 	// Transcribe audio
				

			default:
				break;
		}
	}
}


