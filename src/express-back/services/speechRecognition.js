const speech = require("@google-cloud/speech");
const speechClient = new speech.SpeechClient();

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
