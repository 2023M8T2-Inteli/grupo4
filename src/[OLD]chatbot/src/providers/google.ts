import * as terminal from '../cli/ui';
import { Client, Message, MessageMedia } from 'whatsapp-web.js';
import { TextToSpeechClient } from '@google-cloud/text-to-speech';
import { readFileSync } from 'fs';

export let clientTexToSpeech: TextToSpeechClient;

export function initGoogle() {
  const credentials = {
    type: process.env.TYPE,
    project_id: process.env.PROJECT_ID,
    private_key_id: process.env.PRIVATE_KEY_ID,
    private_key: process.env.PRIVATE_KEY?.replace(/\\n/g, '\n'),
    client_email: process.env.CLIENT_EMAIL,
    client_id: process.env.CLIENT_ID,
    auth_uri: process.env.AUTH_URI,
    token_uri: process.env.TOKEN_URI,
    auth_provider_x509_cert_url: process.env.AUTH_PROVIDER_X509_CERT_URL,
    client_x509_cert_url: process.env.CLIENT_X509_CERT_URL,
    universe_domain: process.env.UNIVERSE_DOMAIN
  };

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
