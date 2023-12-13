// Função que converte a conversa do usuário no Whatsapp para o array que o GPT recebe
import { Message } from 'whatsapp-web.js';

interface ParsedMessage {
  role: 'user' | 'assistant';
  content: string;
}

const transformConversation = (chat: Message[]): ParsedMessage[] => {
  const parsedMsgs: ParsedMessage[] = [];
  for (const wppMessage of chat) {
    const role = wppMessage.id.fromMe ? 'assistant' : 'user';
    parsedMsgs.push({ role, content: wppMessage.body });
  }
  return parsedMsgs;
};

export default transformConversation;