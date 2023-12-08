const fs = require("fs");
const textToSpeech = require("@google-cloud/text-to-speech");

const talkClient = new textToSpeech.TextToSpeechClient();

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
