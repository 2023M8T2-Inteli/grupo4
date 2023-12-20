import { Client, Message, MessageMedia } from 'whatsapp-web.js';
import ExcelJS from 'exceljs';
import { unlink } from 'fs/promises';
import UserService from '../../models/user';
import OrderService from '../../models/order';
import PointService from '../../models/point';
import ToolService from '../../models/tool';
import {
  PrismaClient,
  User as PrismaUser,
  Order as PrismaOrder,
  Role,
} from '@prisma/client';
const prisma = new PrismaClient();
const userService = new UserService(prisma);
const pointService = new PointService(prisma);
const orderService = new OrderService(prisma);
const toolService = new ToolService(prisma);
const { v4: uuidv4 } = require('uuid');

export const delay = async (seconds: number): Promise<void> => {
  return new Promise<void>((resolve) => {
    setTimeout(() => {
      resolve();
    }, seconds);
  });
};

const sendMenu = async (message: Message, client: Client) => {
  try {
    message.reply('*Em que posso te ajudar hoje?*');

    const list =
      '1. Cadastrar novo ponto.🆕\n2. Autorizar acesso.🔑\n3. Exportar relatório do dia.📊';
    client.sendMessage(message.from, list);

    return 'Menu enviado com sucesso!';
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
};

const intentDict = new Map([
  [/\b([Cc]adastrar|[Nn]ovo)|([Pp]onto)\b/gi, 'newPoint'],
  [/\b([Aa]utorizar)|([Aa]cesso)\b/gi, 'newAccess'],
  [/\b([Ee]xportar)|([Rr]elatório|[Rr]elatorio|[Dd]ia)\b/gi, 'newReport'],
]);

const actionDict: { [key: string]: (message: Message, client: Client) => any } =
  {
    newPoint: sendNewPoint,
    newAccess: sendNewAccess,
    newReport: sendNewReport,
  };

const handleAdminRequestMenu = async (message: Message, client: Client) => {
  try {
    sendMenu(message, client);
    await userService.updateRequestUser(message.from, 2);
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
};

const handleAdminProcessRequest = async (message: Message, client: Client) => {
  try {
    let maxMatches = 0;
    let actionToExecute;
    for (const [pattern, action] of intentDict) {
      const matches = pattern.exec(message.body);
      if (matches && matches.length > maxMatches) {
        maxMatches = matches.length;
        actionToExecute = action;
      }
    }
    if (actionToExecute) {
      await actionDict[actionToExecute](message, client);
    } else {
      message.reply('Desculpa, não consegui entender o que você disse.');
      client.sendMessage(message.from, 'Por favor, tente novamente.');
    }
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
};

async function sendNewPoint(message: Message, client: Client) {
  try {
    message.reply('Certo, você deseja cadastrar um novo ponto.');
    userService.updateRequestUser(message.from, 3);
    message.reply(
      'Você pode informar o nome do ponto e as coordenadas X, Y e Z separados por hífen. Exemplo: *Ponto 1 - 1.0 - 2.0 - 3.0*'
    );
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
}

async function sendNewAccess(message: Message, client: Client) {
  try {
    message.reply('Certo, você deseja autorizar um novo acesso.');
    userService.updateRequestUser(message.from, 4);
    client.sendMessage(
      message.from,
      'Você pode informar o número do telefone do usuário que deseja autorizar. Exemplo: *5511999999999*'
    );
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
}

async function sendNewReport(message: Message, client: Client) {
  try {
    message.reply('Certo, você deseja exportar um relatório do dia.');
    client.sendMessage(message.from, 'Aguarde um momento, por favor.');
    const report = await orderService.getOrders();
    if (report) {
      for (let i = 0; i <= report?.length; i++) {
        let user = await userService.getUserbyId(report[i].userId);
        if (user) report[i].userId = user.name;
        let point = await pointService.getPointById(report[i].pointId);
        if (point) report[i].pointId = point.name;
        let tool = await toolService.getToolById(report[i].toolId);
        if (tool) report[i].toolId = tool.name;
      }
    }


    await handleGenerateReport(report, message, client);
    await userService.updateRequestUser(message.from, 1);
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
}

const handleGenerateReport = async (
  orders,
  message: Message,
  client: Client
) => {
  try {
    const workbook = new ExcelJS.Workbook();
    const sheet = workbook.addWorksheet(
      'pedidos' + new Date().toISOString().replace(/[\-\:\.T]/g, '')
    );

    // Cabeçalhos das colunas (ajuste conforme a estrutura de PrismaOrder)
    sheet.columns = [
      { header: 'Id', key: 'id', width: 60 },
      { header: 'Nome do Requerente', key: 'customerName', width: 50 },
      { header: 'Código do Pedido', key: 'code', width: 30 },
      { header: 'Status', key: 'type', width: 30 },
      { header: 'Ferramenta', key: 'tool', width: 50 },
      { header: 'Setor', key: 'point', width: 30 },
    ];

    orders.forEach((order) => {
      sheet.addRow({
        id: order.id,
        code: order.code,
        type: order.type,
        tool: order.toolId,
        customerName: order.userId,
        point: order.pointId,

      });
    });

    const fileName =
      'pedidos_' + new Date().toISOString().replace(/[\-\:\.T]/g, '') + '.xlsx';
    await workbook.xlsx.writeFile(fileName);

    const messageMedia = MessageMedia.fromFilePath(fileName);
    await client.sendMessage(message.from, messageMedia, {
      sendMediaAsDocument: true,
    });
    await unlink(fileName);
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
};

const handleUpdateUserAccess = async (message: Message, client: Client) => {
  try {
    message.reply('Aguarde um momento, por favor.');
    const user = await userService.getUser(message.body + '@c.us');
    if (user == null) {
      message.reply('Usuário não encontrado, por favor, tente novamente.');
      return;
    } else {
      await userService.updateRoletUser(user.cellPhone);
      client.sendMessage(message.from, 'Usuário autorizado com sucesso!');
    }
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
};

const handleNewPoint = async (message: Message, client: Client) => {
  try {
    client.sendMessage(message.from, 'Aguarde um momento, por favor.');

    let newPoint = message.body.split(' - ');
    if (newPoint.length != 4) {
      message.reply('Dados inválidos, por favor, tente novamente.');
      return;
    }
    if (
      isNaN(Number(newPoint[1])) ||
      isNaN(Number(newPoint[2])) ||
      isNaN(Number(newPoint[3]))
    ) {
      message.reply('Dados inválidos, por favor, tente novamente.');
      return;
    }
    if (newPoint.length == 4) {
      const point = await prisma.point.create({
        data: {
          id: uuidv4(),
          name: newPoint[0],
          pointX: parseFloat(newPoint[1]),
          pointY: parseFloat(newPoint[2]),
          pointZ: parseFloat(newPoint[3]),
        },
      });
      pointService.createPoint(point);
      client.sendMessage(message.from, 'Ponto cadastrado com sucesso!');
    }
  } catch (error: any) {
    console.error('An error occured', error);
    message.reply(
      'An error occured, please contact the administrator. (' +
        error.message +
        ')'
    );
  }
};

export {
  handleAdminRequestMenu,
  handleAdminProcessRequest,
  handleUpdateUserAccess,
  handleNewPoint,
};
