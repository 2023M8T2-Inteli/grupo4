const OpenAI = require("openai");
const openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });

async function getOpenAIChatCompletion(messages) {
  return openai.chat.completions.create({
    messages,
    model: "gpt-3.5-turbo",
  });
}

module.exports = { getOpenAIChatCompletion };
