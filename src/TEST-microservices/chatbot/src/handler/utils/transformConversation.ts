// Função que converte a conversa do usuário no Whatsapp para o array que o GPT recebe
import { Message } from 'whatsapp-web.js';

interface ParsedMessage {
  role: 'user' | 'assistant';
  content: string;
}

const transformConversation = (chat: Message[], readyTimestamp: number): ParsedMessage[] => {
  const parsedMsgs: ParsedMessage[] = [];

  const nowTimestamp = Math.floor(+new Date() / 1000);

  for (const wppMessage of chat) {

    // checa se a mensagem foi mandada nas últimas 24h
    if (wppMessage.timestamp < nowTimestamp - 86400) continue;

    // checa se a mensagem foi enviada só depois do bot estar pronto
    if (wppMessage.timestamp < readyTimestamp) continue;


    const role = wppMessage.id.fromMe ? 'assistant' : 'user';
    parsedMsgs.push({ role, content: wppMessage.body });
  }
  console.log("\n-------")
  console.log(parsedMsgs)
  console.log("-------\n")
  return parsedMsgs;
};

export default transformConversation;