import { Client, Message, Events, LocalAuth } from "whatsapp-web.js";
import UserService from "../../models/user";

// Import prisma client
import { PrismaClient, Role } from "@prisma/client";

import { transcribeAudioLocal } from "../../providers/whisper-local";
const prisma = new PrismaClient();

const userService = new UserService(prisma);

const { v4: uuidv4 } = require("uuid");

export const delay = async (seconds: number): Promise<void> => {
	return new Promise<void>((resolve) => {
		setTimeout(() => {
			resolve();
		}, seconds);
	});
};

const sendMenu = async (message: Message, client: Client) => {
	try {
		await message.reply("*Em que posso te ajudar hoje?*");

		await delay(1000);

		const list = "1. Solicitar nova peça\n2. Acompanhar status de um pedido\n3. Cancelar pedido\n4. Falar com um atendente";
		client.sendMessage(message.from, list);

		await delay(1000);

		client.sendMessage(message.from, "Digite apenas o número da opção desejada, por favor.");

		return "Menu enviado com sucesso!";
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const updateRequest = async (message: Message, requestState: number) => {
	try {
		message.reply("Aguarde um momento por favor.");

		await userService.updateRequestUser(message.from, requestState);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleCreateUser = async (message: Message, client: Client, userName: string) => {
	try {
		client.sendMessage(
			message.from,
			`Olá ${userName}, eu sou o Vallet, identifiquei aqui que você não possui cadastro em nosso sistema.`
		);
		await delay(1000);

		client.sendMessage(message.from, "Aguarde um momento por favor.");

		const newUser = {
			id: uuidv4(),
			name: userName,
			cellPhone: message.from,
			requestState: 1,
			createdAt: new Date()
		};

		userService.createAccountUser(newUser);

		await delay(2000);

		message.reply("Conta criada com sucesso!");

		sendMenu(message, client);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleRequestUser = async (message: Message, client: Client) => {
	try {
		sendMenu(message, client);

		userService.updateRequestUser(message.from, 2);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

export const sendContact = async (message: Message, client: Client) => {
	try {
		const user = await userService.getAdmin();
		if (user) {
			const contact = await client.getContactById(user?.cellPhone);
			client.sendMessage(message.from, contact);
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleTrackOrder = async (message: Message, client: Client, userName: string) => {
	try {
		
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}

}
const handleRequestMenu = async (message: Message, client: Client) => {
	try {
		switch (message.body) {
			case "1":
				await updateRequest(message, 1);
				message.reply("Serviço disponível em breve :)!!!");
				break;

			case "2":
				updateRequest(message, 1);
				message.reply("No momento, não temos nenhum pedido registrado em seu nome.");

				break;

			case "3":
				updateRequest(message, 1);
				message.reply("No momento, não temos nenhum pedido registrado em seu nome.");
				
				break;

			case "4":
				updateRequest(message, 1);

				message.reply("Para falar com um atendente, por favor envie uma mensagem para o número:");
				await delay(1000);
				sendContact(message, client);


				break;

			default:
				message.reply("Opção inválida, por favor digite apenas o número da opção desejada.");
				break;
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleRequestNewPiece = async (message: Message, client: Client) => {
	if (message.hasMedia) {
		const media = await message.downloadMedia();

		// Ignore non-audio media
		if (!media || !media.mimetype.startsWith("audio/")) return;

		// Convert media to base64 string
		const mediaBuffer = Buffer.from(media.data, "base64");


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
	}
};

export { updateRequest, handleCreateUser, handleRequestUser, handleRequestNewPiece, handleRequestMenu };
