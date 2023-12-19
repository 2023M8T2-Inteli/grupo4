const express = require("express");
const multer = require("multer");
const dotenv = require("dotenv");
const axios = require("axios");
const {
  convertTextToSpeech,
  recognizeSpeech,
  getOpenAIChatCompletion,
} = require("../services");

dotenv.config();

const router = express.Router();
const storage = multer.memoryStorage();
const upload = multer({ storage });

async function fetchData(url) {
  try {
    const response = await axios.get(url);
    const data = {
      now: response.data[0],
      queue: response.data.slice(1),
      history: await axios.get("http://localhost:5000/orders/history").data,
    };
    return data;
  } catch (error) {
    console.error("Error:", error);
    throw error;
  }
}

function filterJson(inputJson, properties) {
  const filteredData = {};

  for (const prop of properties) {
    if (typeof prop === "string" && inputJson[prop] !== undefined) {
      filteredData[prop] = inputJson[prop];
    } else if (typeof prop === "object") {
      const [key, nestedProps] = Object.entries(prop)[0];
      if (inputJson[key] && typeof inputJson[key] === "object") {
        filteredData[key] = filterJson(inputJson[key], nestedProps);
      }
    }
  }

  return filteredData;
}


async function buildContext() {
  const data = await fetchData("http://localhost:5000/orders/queue");
  const selectedProperties = [
    { tool: ["name"] },
    { user: ["name"] },
    { point: ["name"] },
    "createdAt",
  ];

  const filteredArray = data?.queue.map((item) =>
    filterJson(item, selectedProperties)
  );

  const filteredHistory = data?.history.map((item) =>
    filterJson(item, selectedProperties)
  );

  return {
    role: "system",
    content: `
    Você é um assistente para delivery de peças do almoxarifado na Ambev. Seu nome é Vallet. Responda as perguntas de forma simpática e divertida. Na primeira resposta, se apresente e diga o que pode fazer. Seja conciso. O "point" nos dados é o lugar para onde o itens será entregue, onde se encontra a pessoa que fez pedido. Presta atenção para não confundir fila com histórico. Se a pessoa perguntar o que falta, é a fila, se ela perguntar o que já foi entregue, é o histórico.
    
    PEDIDO SENDO EXECUTADO AGORA: ${JSON.stringify(
      filterJson(data?.now, selectedProperties)
    )}
    Nº DE ITENS NA FILA (PEDIDOS QUE FALTAM SER ENTREGUES): ${
      data?.queue.length
    }
    FILA (PEDIDOS QUE FALTAM SER ENTREGUES): ${JSON.stringify(filteredArray)}
    HISTÓRICO: ${JSON.stringify(filteredHistory)}
    `,
  };
}

let conversationHistory = [buildContext()];

router.post("/speak", upload.single("audioFile"), async (req, res) => {
  try {
    conversationHistory[0] = await buildContext();
  
    const audioBuffer = req.file.buffer;
    const transcription = await recognizeSpeech(audioBuffer);

    conversationHistory.push({ role: "user", content: transcription });

    console.log("Conversation history:", conversationHistory);
    const chatCompletion = await getOpenAIChatCompletion(conversationHistory);
    const modelResponse = chatCompletion.choices[0].message.content;

    conversationHistory.push({ role: "assistant", content: modelResponse });

    let emotion = await getOpenAIChatCompletion([
      {
        role: "system",
        content:
          "Leia o texto abaixo e defina a emoção principal dele entre happy, superhappy, e sad. Se o usuário elogiar ou agradecer, deve ser superhappy. Se a resposta for um pedido de desculpas, deve ser triste.: " +
          modelResponse,
      },
    ]);

    emotion = emotion.choices[0].message.content;

    const { base64Audio } = await convertTextToSpeech(modelResponse);

    res.status(200).json({
      message: "Audio file received, transcribed, and processed successfully",
      transcription,
      modelResponse,
      base64Audio,
      emotion,
    });
  } catch (error) {
    console.error("Error processing audio:", error);
    res.status(500).json({ error: "Internal server error" });
  }
});

module.exports = router;
