const fs = require("fs");
const textToSpeech = require("@google-cloud/text-to-speech");
require("dotenv").config();
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

const talkClient = new textToSpeech.TextToSpeechClient({credentials});

async function convertTextToSpeech(text) {
  const speechRequest = {
    input: { text },
    voice: {
      languageCode: "pt-BR",
      name: "pt-BR-Neural2-B",
    },
    audioConfig: { audioEncoding: "MP3", pitch: 13, speakingRate: 1.2 },
  };

  const [speechResponse] = await talkClient.synthesizeSpeech(speechRequest);
  const fileName = "hmm.mp3";
  fs.writeFileSync(fileName, speechResponse.audioContent);

  const base64Audio = speechResponse.audioContent.toString("base64");

  return { fileName, base64Audio };
}

module.exports = { convertTextToSpeech };
