import { Chat, Message } from 'whatsapp-web.js';
import { WhatsappService } from '../../whatsapp/whatsapp.service';

export const validate_message = async (
  message: Message,
  botReadyTimestamp: number | null,
): Promise<boolean> => {
  const messageFrom: Chat = await message.getChat();

  if (messageFrom.isGroup) {
    return false;
  } else if (botReadyTimestamp == null) {
    return false;
  } else if (+new Date(message.timestamp) < botReadyTimestamp) {
    return false;
  }

  return true;
};

export const check_out = async (
  message: Message,
  whatsAppClient: WhatsappService
): Promise<boolean> => {
  if (message.body == '!sair' || message.body == '!Sair') {
    await whatsAppClient.sendMessage(message.from, 'Até mais!');
    // userService.updateRequestUser(message.from, 1);
    return true;
  }

  return false;
};
