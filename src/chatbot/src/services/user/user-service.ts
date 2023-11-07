import { Client, Message, Events, LocalAuth } from "whatsapp-web.js";
import UserService from "../../models/user";

// Import prisma client
import { PrismaClient } from "@prisma/client";
const prisma = new PrismaClient();

const userService = new UserService(prisma);

const { v4: uuidv4 } = require("uuid");

const delay = async (seconds: number): Promise<void> => {
	return new Promise<void>((resolve) => {
		setTimeout(() => {
			resolve();
		}, seconds);
	});
};

const sendMenu = async (message: Message, client: Client) => {
	try {
		await message.reply("*Em que posso te ajudar hoje?*");

		await delay(500);
		
		const list = "1. Solicitar nova peça\n2. Acompanhar status de um pedido\n 3. Cancelar pedido\n4. Falar com um atendente";
		client.sendMessage(message.from, list);

		await delay(500);

		client.sendMessage(message.from, "Digite apenas o número da opção desejada, por favor.");

		return "Menu enviado com sucesso!";
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const updateRequest = async (message: Message, prisma: PrismaClient, requestState: number) => {
	try{
		const user = await userService.getUser(message.from);
		if(user == null){
			message.reply("Não foi possível encontrar seu cadastro, por favor digite seu nome completo.");
			return;
		}
		await userService.updateUser({...user, requestState});

	}catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}

}

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
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleRequestNewPiece = async (message: Message, client: Client) => {};

export {updateRequest, handleCreateUser, handleRequestUser, handleRequestNewPiece};
