import { PrismaClient } from "@prisma/client";
import UserService from "../models/user";
import { Client, Message, List } from "whatsapp-web.js";
import { botReadyTimestamp } from "../index";
import * as cli from "../cli/ui";
// Config & Constants
import config from "../config";
import { handleCreateUser, handleRequestMenu, handleRequestUser, updateRequest } from "../services/user/user-service";
import { handleMessageAIConfig, getConfig, executeCommand } from "../handlers/ai-config";

// // Speech API & Whisper
import { TranscriptionMode } from "../types/transcription-mode";
import { transcribeRequest } from "../providers/speech";
import { transcribeAudioLocal } from "../providers/whisper-local";
import { transcribeWhisperApi } from "../providers/whisper-api";
import { transcribeOpenAI } from "../providers/openai";

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
				// 	// Transcribe audio
				if (message.hasMedia) {
					const media = await message.downloadMedia();

					// Ignore non-audio media
					if (!media || !media.mimetype.startsWith("audio/")) return;

					// Check if transcription is enabled (Default: false)
					if (!getConfig("transcription", "enabled")) {
						cli.print("[Transcription] Received voice messsage but voice transcription is disabled.");
						return;
					}

					// Convert media to base64 string
					const mediaBuffer = Buffer.from(media.data, "base64");

					// Transcribe locally or with Speech API
					const transcriptionMode = getConfig("transcription", "mode");

					cli.print(`[Transcription] Transcribing audio with "${transcriptionMode}" mode...`);

					let res;

					await transcribeAudioLocal(mediaBuffer);

					// switch (transcriptionMode) {
					// 	case TranscriptionMode.Local:
					// 		res = await transcribeAudioLocal(mediaBuffer);
					// 		break;
					// 	case TranscriptionMode.OpenAI:
					// 		res = await transcribeOpenAI(mediaBuffer);
					// 		break;
					// 	case TranscriptionMode.WhisperAPI:
					// 		res = await transcribeWhisperApi(new Blob([mediaBuffer]));
					// 		break;
					// 	case TranscriptionMode.SpeechAPI:
					// 		res = await transcribeRequest(new Blob([mediaBuffer]));
					// 		break;
					// 	default:
					// 		cli.print(`[Transcription] Unsupported transcription mode: ${transcriptionMode}`);
					// }
					const { text: transcribedText, language: transcribedLanguage } = res;

					// Check transcription is null (error)
					if (transcribedText == null) {
						message.reply("Não consegui entender o que você disse.");
						return;
					}

					// Check transcription is empty (silent voice message)
					if (transcribedText.length == 0) {
						message.reply("Não consegui entender o que você disse.");
						return;
					}


					// Reply with transcription
					const reply = `You said: ${transcribedText}${transcribedLanguage ? " (language: " + transcribedLanguage + ")" : ""}`;
					message.reply(reply);
				}else{

				}

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
