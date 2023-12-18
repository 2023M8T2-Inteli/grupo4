import * as terminal from '../cli/ui';
import { Client, Message, MessageMedia } from 'whatsapp-web.js';
import { TextToSpeechClient } from '@google-cloud/text-to-speech';
import { readFileSync } from 'fs';

export let clientTexToSpeech: TextToSpeechClient;

export function initGoogle() {
  const credentials = JSON.parse(readFileSync('./json.json', 'utf-8'));
  clientTexToSpeech = new TextToSpeechClient({
    credentials,
  });
}

export async function speechGoogle(
  message: Message,
  client: Client,
  text: string | undefined
): Promise<String> {
  let response;
  try {
    response = await clientTexToSpeech.synthesizeSpeech({
      input: { text: text },
      voice: { languageCode: 'pt-BR', ssmlGender: 'FEMALE' },
      audioConfig: { audioEncoding: 'MP3' },
    });
  } catch (e) {
    terminal.print(e);
    return '';
  } finally {
    const audioBase64 = Buffer.from(response[0].audioContent, 'binary').toString('base64');
    const messageMedia = new MessageMedia('audio/mp3', audioBase64);
    await client.sendMessage(message.from, messageMedia, {
      sendAudioAsVoice: true,
    });
    return response.audioContent;
  }
}
