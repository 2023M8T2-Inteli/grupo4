import { Client, Message, Events, LocalAuth, List } from "whatsapp-web.js";
import UserService from "../../models/user";

// Import prisma client
import { PrismaClient } from "@prisma/client";
const prisma = new PrismaClient();

const userService = new UserService(prisma);

const { v4: uuidv4 } = require("uuid");

const sendMenu = async (message: Message, client: Client) => {};

const handleCreateUser = async (message: Message, client: Client, userName: string) => {
	try {
		setTimeout(function () {
			client.sendMessage(
				message.from,
				`Olá ${userName}, eu sou o Vallet, identifiquei aqui que você não possui cadastro em nosso sistema.`
			);
		}, 500);

		setTimeout(function () {
			client.sendMessage(message.from, "Aguarde um momento por favor.");
		}, 1000);

		const newUser = {
			id: uuidv4(),
			name: userName,
			cellPhone: message.from,
			requestState: 1,
			createdAt: new Date()
		};

		userService.createAccountUser(newUser);

		setTimeout(function () {
			message.reply("Conta criada com sucesso!");
		}, 1000);

		const sections = [{ title: "Opções disponíveis", rows: [{ title: "Solicitar peça" }, { title: "Alterar nome cadastrado" }] }];
		const menu = new List("Em que posso te ajudar? Selecione uma opção no menu:", "Menu", sections, "Menu de opções:");
		await client.sendMessage(message.from, menu);

	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleRequestUser = async (message: Message, client: Client) => {
	try {
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

export { handleCreateUser, handleRequestUser };
