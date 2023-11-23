import { Client, Message } from "whatsapp-web.js";
import UserService from "../../models/user";
import OrderService from "../../models/order";
import PointService from "../../models/point";
import { PrismaClient, User as PrismaUser, Role } from "@prisma/client";
import { handleMessageGPT } from "../../handlers/gpt";
import { transcribeOpenAI } from "../../providers/openai";
const prisma = new PrismaClient();
const userService = new UserService(prisma);
const orderService = new OrderService(prisma);
const pointService = new PointService(prisma);
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

		const list = "*1.* Cadastrar novo ponto.🆕\n*2.* Autorizar acesso.🔑";
		client.sendMessage(message.from, list);

		await delay(1000);

		client.sendMessage(message.from, "Digite apenas o número da opção desejada, por favor.");

		return "Menu enviado com sucesso!";
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleAdminRequestMenu = async (message: Message, client: Client) => {
	try {
		sendMenu(message, client);
		await userService.updateRequestUser(message.from, 2);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleAdminProcessRequest = async (message: Message, client: Client) => {
	try {
		switch (message.body) {
			case "1":
				message.reply("Certo, você deseja cadastrar um novo ponto.");
				userService.updateRequestUser(message.from, 3);
				message.reply("Por favor, digite as informações do novo ponto como no exemplo abaixo: \n nome - 1,0 - 1,0 - 0,0");
				break;
			case "2":
				message.reply("Certo, você deseja autorizar acesso.");
				userService.updateRequestUser(message.from, 4);
				message.reply("Por favor, digite o número de telefone do usuário que deseja autorizar. *Exemplo: 5511999999999*");
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

const handleUpdateUserAccess = async (message: Message, client: Client) => {
	try {
		message.reply("Aguarde um momento, por favor.");
		const user = await userService.getUser(message.body + "@c.us");
		if (user == null) {
			message.reply("Usuário não encontrado, por favor, tente novamente.");
			return;
		} else {
			await userService.updateRoletUser(user.cellPhone);
			client.sendMessage(message.from, "Usuário autorizado com sucesso!");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleNewPoint = async (message: Message, client: Client) => {
	try {
		
		client.sendMessage(message.from, "Aguarde um momento, por favor.");

		let newPoint = message.body.split(" - ");
		if (newPoint.length != 4) {
			message.reply("Dados inválidos, por favor, tente novamente.");
			return;
		}
		if (isNaN(Number(newPoint[1])) || isNaN(Number(newPoint[2])) || isNaN(Number(newPoint[3]))) {
			message.reply("Dados inválidos, por favor, tente novamente.");
			return;
		}
		if(newPoint.length == 4){
			const point = await prisma.point.create({
				data: {
					id: uuidv4(),
					name: newPoint[0],
					pointX: parseFloat(newPoint[1]),
					pointY: parseFloat(newPoint[2]),
					pointZ: parseFloat(newPoint[3])
				}
			});
			pointService.createPoint(point);
			client.sendMessage(message.from, "Ponto cadastrado com sucesso!");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

export { handleAdminRequestMenu, handleAdminProcessRequest, handleUpdateUserAccess, handleNewPoint };
