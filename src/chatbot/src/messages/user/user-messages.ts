import { Client, Message } from "whatsapp-web.js";
import UserService from "../../models/user";
import OrderService from "../../models/order";
import ToolService from "../../models/tool";
import { PrismaClient, User as PrismaUser, Role } from "@prisma/client";
import { getPointOpenAI } from "../../providers/openai";
import * as terminal from "../../cli/ui";
import PointService from "../../models/point";
const prisma = new PrismaClient();
const userService = new UserService(prisma);
const orderService = new OrderService(prisma);
const toolService = new ToolService(prisma);
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
		client.sendMessage(message.from, "*Em que posso te ajudar hoje?*");

		await delay(1000);

		const list =
			"*1.* Solicitar nova peça. 🆕\n*2.* Acompanhar status de um pedido. 📦\n*3.* Acompanhar pedidos em aberto. 📑\n*4.* Cancelar pedido. ❌\n*5.* Falar com um atendente. 💬\n*6.* Alterar nome cadastrado. ✏️";
		client.sendMessage(message.from, list);
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
			role: ["LEAD"],
			createdAt: new Date()
		};

		userService.updateAccountUser(newUser);
		handleLeadAcess(message, client);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleUpdateName = async (message: Message, client: Client) => {
	try {
		message.reply(`Certo, ${message.body}!`);
		await delay(1000);
		userService.updateName(message.from, message.body);
		handleLeadAcess(message, client);
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

const intentDict = new Map([
	[/\b([Nn]ova)|([Pp]eça)\b/gi, "newOrder"],
	[/\b([Ss]tatus)|([Pp]edido)\b/gi, "statusOrder"],
	[/\b([Pp]edidos)|([Aa]berto)|([Aa]bertos)\b/gi, "openOrders"],
	[/\b([Cc]ancelar)|([Pp]edido)\b/gi, "cancelOrder"],
	[/\b([Ff]alar)|([Aa]tendente)|([Cc]om)\b/gi, "contact"],
	[/\b([Aa]lterar)|([Nn]ome)|([Cc]adastrado)\b/gi, "changeName"]
]);

// Define the action dictionaryconst
const actionDict: { [key: string]: (message: Message, client: Client) => any } = {
	"newOrder": sendNewOrder,
	"statusOrder": sendStatusOrder,
	"openOrders": sendOpenOrders,
	"cancelOrder": sendCancelOrder,
	"contact": sendContact,
	"changeName": sendChangeName
};

async function sendNewOrder(message: Message, client: Client) {
	try {
		message.reply("Certo, você deseja solicitar uma nova peça.");
		userService.updateRequestUser(message.from, 3);
		message.reply("Você pode me dizer onde você está?");
		const points = await pointService.getPoints();
		let listPoints = "";
		if (points && points.length > 0) {
			for (const point of points) {
				listPoints += "*" + point.name + "*\n";
			}
			message.reply(listPoints);
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

async function sendStatusOrder(message: Message, client: Client) {
	try {
		message.reply("Certo, você deseja acompanhar o status de um pedido.");
		userService.updateRequestUser(message.from, 4);
		message.reply("Por favor, digite o número do pedido que deseja acompanhar.");
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

async function sendOpenOrders(message: Message, client: Client) {
	try {
		message.reply("Certo, você deseja acompanhar seus pedidos em aberto.");
		handleOpenOrder(message, client);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

async function sendCancelOrder(message: Message, client: Client) {
	try {
		message.reply("Certo, você deseja cancelar um pedido.");
		userService.updateRequestUser(message.from, 5);
		message.reply("Por favor, digite o número do pedido que deseja cancelar.");
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

async function sendContact(message: Message, client: Client) {
	try {
		message.reply("Certo, você deseja falar com um atendente.");
		handleSendContact(message, client);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

async function sendChangeName(message: Message, client: Client) {
	try {
		message.reply("Certo, você deseja alterar seu nome cadastrado.");
		userService.updateRequestUser(message.from, 6);
		client.sendMessage(message.from, "Por favor, digite seu nome completo, por favor.");
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
}

const handleProcessRequest = async (message: Message, client: Client) => {
	try {
		let maxMatches = 0;
		let actionToExecute;
		for (const [pattern, action] of intentDict) {
			const matches = pattern.exec(message.body);
			terminal.print(`[Intent] ${pattern} -> ${matches ? matches.length : 0} matches`);
			if (matches && matches.length > maxMatches) {
				maxMatches = matches.length;
				actionToExecute = action;
			}
		}
		if (actionToExecute) {
			await actionDict[actionToExecute](message, client);
		} else {
			message.reply("Desculpa, não consegui entender o que você disse.");
			client.sendMessage(message.from, "Por favor, tente novamente.");
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
			message.reply("*Pedido:* " + order.code + "\n*Status:* " + order.type + "\n*Data:* " + order.createdAt);
			await delay(1000);
			client.sendMessage(message.from, "Aguarde um momento por favor.");
			await delay(1000);

			orderService.cancelOrder(message);

			message.reply("Pedido cancelado com sucesso!");
			userService.updateRequestUser(message.from, 1);
		} else {
			message.reply("Pedido não encontrado, por favor, digite o número do pedido que deseja cancelar.");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleStatusOrder = async (message: Message, client: Client) => {
	try {
		client.sendMessage(message.from, "Certo. Aguarde um momento por favor.");

		const order = await orderService.getOrder(message);
		if (order) {
			message.reply("*Pedido:* " + order.code + "\n*Status:* " + order.type + "\n*Data:* " + order.createdAt);
			await delay(1000);

			userService.updateRequestUser(message.from, 1);
		} else {
			message.reply("Pedido não encontrado, por favor, digite o número do pedido que deseja cancelar.");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleOpenOrder = async (message: Message, client: Client) => {
	try {
		client.sendMessage(message.from, "Certo. Aguarde um momento por favor.");
		const orders = await orderService.getOpenOrder(message);
		if (orders && orders.length > 0) {
			for (const order of orders) {
				message.reply("*Pedido:* " + order.code + "\n*Status:* " + order.type + "\n*Data:* " + order.createdAt);
				await delay(1000);
			}
			userService.updateRequestUser(message.from, 1);
		} else {
			message.reply("No momento você não possui pedidos em aberto.");
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

const handleNewOrder = async (message: Message, client: Client) => {
	try {
		// if (message.hasMedia) {
		// 	const media = await message.downloadMedia();
		// 	const transcriptionMode = getConfig("transcription", "mode");
		// 	terminal.print(`[Transcription] Transcribing audio with "${transcriptionMode}" mode...`);
		// 	// Convert media to base64 string
		// 	const mediaBuffer = Buffer.from(media.data, "base64");
		// 	let response = await transcribeOpenAI(mediaBuffer);
		// 	const { text: transcribedText, language: transcribedLanguage } = response;
		// 	// Check transcription is null (error)
		// 	if (transcribedText == null || transcribedText.length == 0) {
		// 		message.reply("Desculpa, não consegui entender o que você disse.");
		// 		client.sendMessage(message.from, "Por favor, teria como me mandar um áudio novamente.");
		// 		return;
		// 	}
		// 	const chat_response = await handleMessageGPT(message, transcribedText);

		// 	console.log(chat_response);
		// }
		if (message.body) {
			const points = await pointService.getPoints();
			const chat_response = await getPointOpenAI(message, points);
			console.log(chat_response);
		}
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};

export {
	handleCreateUser,
	handleUpdateUser,
	handleLeadAcess,
	handleRequestMenu,
	handleProcessRequest,
	handleCancelOrder,
	handleStatusOrder,
	handleOpenOrder,
	handleNewOrder,
	handleUpdateName
};
