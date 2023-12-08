const express = require("express");
const multer = require("multer");
const dotenv = require("dotenv");
const { convertTextToSpeech } = require("../services/speechHelper");
const { recognizeSpeech } = require("../services/speechRecognition");
const { getOpenAIChatCompletion } = require("../services/openAiHelper");
const axios = require("axios");

dotenv.config();

const router = express.Router();
const storage = multer.memoryStorage();
const upload = multer({ storage: storage });

async function updateStatus() {
  try {
    const response = await axios.get("http://localhost:5000/orders/queue");
    const data = {
      now: response.data[0],
      queue: response.data.slice(1),
    };
    const history = await axios.get("http://localhost:5000/orders/history");
    data['history'] = history.data;
    return data;
  } catch (error) {
    console.error("Error:", error);
    throw error; // Throw the error to propagate it
  }
}

function filterJson(inputJson, properties) {
  const result = {};

  function getFilteredData(data, props) {
    const filteredData = {};
    for (const prop of props) {
      if (typeof prop === "object") {
        // Nested property
        const [key, nestedProps] = Object.entries(prop)[0];
        if (data[key] && typeof data[key] === "object") {
          filteredData[key] = getFilteredData(data[key], nestedProps);
        }
      } else if (data[prop] !== undefined) {
        // Simple property
        filteredData[prop] = data[prop];
      }
    }
    return filteredData;
  }

  return getFilteredData(inputJson, properties);
}

async function buildContext() {
  let data = await updateStatus(); // Declare data using let or const
  const selectedProperties = [
    { tool: ["name"] },
    { user: ["name"] },
    { point: ["name"] },
    "createdAt",
  ];

  // Use Array.map to apply filterJson to each item in the array
  const filteredArray = data?.queue.map((item) =>
    filterJson(item, selectedProperties)
  );

  const filteredHistory = data?.history.map((item) =>
    filterJson(item, selectedProperties)
  );

  return {
    role: "system",
    content: `Você é um assistente para delivery de peças do almoxarifado na Ambev. Seu nome é Vallet. Responda as perguntas de forma simpática e divertida. Na primeira resposta, se apresente e diga o que pode fazer. Seja conciso. O "point" nos dados é o lugar para onde o itens será entregue, onde se encontra a pessoa que fez pedido. Presta atenção para não confundir fila com histórico. Se a pessoa perguntar o que falta, é a fila, se ela perguntar o que já foi entregue, é o histórico.
    
    PEDIDO SENDO EXECUTADO AGORA: ${JSON.stringify(
      filterJson(data?.now, [
        { tool: ["name"] },
        { user: ["name"] },
        { point: ["name"] },
      ])
    )}
    Nº DE ITENS NA FILA (PEDIDOS QUE FALTAM SER ENTREGUES): ${data?.queue.length}
    FILA (PEDIDOS QUE FALTAM SER ENTREGUES): ${JSON.stringify(filteredArray)}
    HISTÓRICO: ${JSON.stringify(filteredHistory)}
    `,
  };
}

let conversationHistory = [buildContext()];

router.post("/speak", upload.single("audioFile"), async (req, res) => {
  try {
    conversationHistory[0] = await buildContext();
    console.log(conversationHistory[0]);
    const audioBuffer = req.file.buffer;
    const transcription = await recognizeSpeech(audioBuffer);

    conversationHistory.push({ role: "user", content: transcription });

    const chatCompletion = await getOpenAIChatCompletion(conversationHistory);
    const modelResponse = chatCompletion.choices[0].message.content;

    conversationHistory.push({ role: "assistant", content: modelResponse });

    const { base64Audio } = await convertTextToSpeech(modelResponse);

    res.status(200).json({
      message: "Audio file received, transcribed, and processed successfully",
      transcription,
      modelResponse,
      base64Audio,
    });
  } catch (error) {
    console.error("Error processing audio:", error);
    res.status(500).json({ error: "Internal server error" });
  }
});

module.exports = router;
