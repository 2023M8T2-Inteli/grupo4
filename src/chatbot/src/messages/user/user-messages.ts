import { Client, Message, Events, LocalAuth } from "whatsapp-web.js";
import UserService from "../../models/user";

// Import prisma client
import { PrismaClient, User as PrismaUser, Role } from "@prisma/client";

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

const handleCreateUser = async (message: Message, client: Client) => {
	try {
		message.reply(`Olá, eu sou o Vallet, tudo bem?`);
		await delay(1000);

        client.sendMessage(message.from, "Para que eu possa te ajudar, preciso que você crie uma conta em nosso sistema.");

        await delay(1000);

        client.sendMessage(message.from, "Para isso, preciso que você me informe seu nome completo, por favor.");

		const newUser: PrismaUser = {
			id: uuidv4(),
			name: "",
			cellPhone: message.from,
			requestState: 0,
            role: [Role.LEAD],
			createdAt: new Date()
		};

		userService.createAccountUser(newUser);


	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleUpdateUser = async (message: Message, client: Client) => {
    try {
		message.reply(`Certo, ${message.body}!`);
		await delay(1000);

        client.sendMessage(message.from, "Aguarde um momento por favor.");

		const newUser: PrismaUser = {
			id: uuidv4(),
			name: message.body,
			cellPhone: message.from,
			requestState: 1,
            role: [Role.LEAD],
			createdAt: new Date()
		};

		userService.updateAccountUser(newUser);


	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

const handleLeadAcess = async (message: Message, client: Client) => {
    try {
		const user = await userService.getAdmin();
		if (user) {
            message.reply(`No momento você não possui permissão para acessar o sistema, ${user.name} irá te ajudar!`);
            await delay(1000);
            client.sendMessage(message.from, "Por favor, solicite a ele que te dê acesso ao sistema.");
			const contact = await client.getContactById(user?.cellPhone);
			client.sendMessage(message.from, contact);
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

export{handleCreateUser, handleUpdateUser, handleLeadAcess}