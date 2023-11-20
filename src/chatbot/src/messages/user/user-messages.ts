import { Client, Message, Events, LocalAuth } from "whatsapp-web.js";
import UserService from "../../models/user";
import OrderService from "../../models/order";

// Import prisma client
import { PrismaClient, User as PrismaUser, Role } from "@prisma/client";

import { transcribeAudioLocal } from "../../providers/whisper-local";
const prisma = new PrismaClient();

const userService = new UserService(prisma);
const orderService = new OrderService(prisma);

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

		const list =
			"1. Solicitar nova peça\n2. Acompanhar status de um pedido\n3. Cancelar pedido\n4. Falar com um atendente\n5. Alterar nome cadastrado";
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
};

const handleLeadAcess = async (message: Message, client: Client) => {
	try {
		message.reply(`No momento você não possui permissão para acessar o sistema!`);
		await delay(1000);
		message.reply("Solicite a um atendente que te dê acesso ao sistema.");
		const user = await userService.getAdmin();
		if (user) {
			client.sendMessage(message.from, "Por favor, solicite a ele que te dê acesso ao sistema.");
			const contact = await client.getContactById(user?.cellPhone);
			client.sendMessage(message.from, contact);
		} else {
			await delay(1000);
			client.sendMessage(message.from, "No momento não temos atendentes disponíveis, por favor, tente novamente mais tarde.");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleRequestMenu = async (message: Message, client: Client) => {
	try {
		sendMenu(message, client);
		await userService.updateRequestUser(message.from, 2);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleProcessRequest = async (message: Message, client: Client) => {
	try {
		switch (message.body) {
			case "1":
				message.reply("Certo, você deseja solicitar uma nova peça.");
				userService.updateRequestUser(message.from, 3);
				break;
			case "2":
				message.reply("Certo, você deseja acompanhar o status de um pedido.");
				userService.updateRequestUser(message.from, 4);
				break;
			case "3":
				message.reply("Certo, você deseja cancelar um pedido.");

				userService.updateRequestUser(message.from, 5);

				message.reply("Por favor, digite o número do pedido que deseja cancelar.");
				break;
			case "4":
				message.reply("Certo, você deseja falar com um atendente.");
				handleSendContact(message, client);
				break;
			default:
				message.reply("Opção inválida, por favor, digite apenas o número da opção desejada.");
				break;
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleSendContact = async (message: Message, client: Client) => {
	try {
		const user = await userService.getAdmin();
		if (user) {
			const contact = await client.getContactById(user?.cellPhone);
			client.sendMessage(message.from, contact);
		} else {
			message.reply("No momento não temos atendentes disponíveis, por favor, tente novamente mais tarde.");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleCancelOrder = async (message: Message, client: Client) => {
	try {
		const order = await orderService.getOrder(message);
		if (order) {
			message.reply("Pedido: " + order.id + "\nStatus: " + order.type + "\nData: " + order.createdAt);
			await delay(1000);
			client.sendMessage(message.from,"Aguarde um momento por favor.");
			await delay(1000);

			orderService.cancelOrder(message);

			message.reply("Pedido cancelado com sucesso!");

		} else {
			message.reply("Pedido não encontrado, por favor, digite o número do pedido que deseja cancelar.");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

export { handleCreateUser, handleUpdateUser, handleLeadAcess, handleRequestMenu, handleProcessRequest, handleCancelOrder };
