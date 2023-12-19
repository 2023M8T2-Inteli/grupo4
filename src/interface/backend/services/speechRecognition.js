const speech = require("@google-cloud/speech");
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

const speechClient = new speech.SpeechClient(credentials);


async function recognizeSpeech(audioBuffer) {
  const config = {
    encoding: "MP3",
    sampleRateHertz: 16000,
    languageCode: "pt-BR",
  };

  const request = {
    audio: { content: audioBuffer },
    config: config,
  };

  const [response] = await speechClient.recognize(request);

  const transcription = response.results
    .map((result) => result.alternatives[0].transcript)
    .join("\n");

  return transcription;
}

module.exports = { recognizeSpeech };
