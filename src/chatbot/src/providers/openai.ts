import fs from "fs";
import os from "os";
import path from "path";
import { randomUUID } from "crypto";
import { ChatGPTAPI } from "chatgpt";
import { Configuration, OpenAIApi } from "openai";
import ffmpeg from "fluent-ffmpeg";
import { blobFromSync, File } from "fetch-blob/from.js";
import config from "../config";
import exp from "constants";
import { Message } from "whatsapp-web.js";
import { json } from "stream/consumers";
import {io}  from "socket.io-client"

export let openai: OpenAIApi;

export function initOpenAI() {
    const apiKey = process.env.OPENAI_API_KEY;
    if (!apiKey) {
        throw new Error("A chave de API do OpenAI não está definida nas variáveis de ambiente.");
    }

    openai = new OpenAIApi(
        new Configuration({
            apiKey: apiKey
        })
    );
}

export async function getPointOpenAI(message: Message, points) {
	let prompt =  "Responda a pergunta abaixo com base no contexto para encontrar as coordenadas do lugar. Fique atento para possíveis variações no nome quando o usuário perguntar.Sempre responda na língua que o usuário se comunicar. Sempre dê as coordenadas no formato ([x], [y], [z])"
	let jsonPoints = JSON.stringify(points)
		
	let question = `Lista de pontos: ${jsonPoints}. Pergunta: Identifique a responsta do usuário com base na lista de pontos Resposta: ${message.body} e depois coloque as coordenadas do ponto em formato de float.`


	const response = await openai.createChatCompletion({
		model: 'gpt-4',
    	messages: [{role: 'system', content: prompt}, { role: 'user', content: question } ],
	
	});

	let pointResponse = response.data.choices[0].message?.content;

	const regex: RegExp = /-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?/gi;
    const match = pointResponse?.match(regex);
	const socket = io("http://10.128.68.115:3000");
    if (match) {
		match.forEach(coordinateString => {
			// Splitting the matched string into individual numbers
			const parts = coordinateString.split(',').map(part => parseFloat(part.trim()));
			const [x, y, z] = parts;
			socket.emit("enqueue", {x, y, z});
			message.reply(pointResponse as any);

		});
    }
	else{
		message.reply("Não consegui encontrar o ponto. Tente novamente.")
	}
}

// export async function transcribeOpenAI(audioBuffer: Buffer): Promise<{ text: string; language: string }> {
// 	const url = config.openAIServerUrl;
// 	let language = "pt-BR";

// 	const tempdir = os.tmpdir();
// 	const oggPath = path.join(tempdir, randomUUID() + ".ogg");
// 	const wavFilename = randomUUID() + ".wav";
// 	const wavPath = path.join(tempdir, wavFilename);
// 	fs.writeFileSync(oggPath, audioBuffer);
// 	try {
// 		await convertOggToWav(oggPath, wavPath);
// 	} catch (e) {
// 		fs.unlinkSync(oggPath);
// 		return {
// 			text: "",
// 			language
// 		};
// 	}

// 	// FormData
// 	const formData = new FormData();
// 	formData.append("file", new File([blobFromSync(wavPath)], wavFilename, { type: "audio/wav" }));
	
// 	formData.append("model", "whisper-1");
// 	if (config.transcriptionLanguage) {
// 		formData.append("language", config.transcriptionLanguage);
// 		language = config.transcriptionLanguage;
// 	}

// 	const headers = new Headers();
// 	headers.append("Authorization", `Bearer ${getConfig("gpt", "apiKey")}`);

// 	// Request options
// 	const options = {
// 		method: "POST",
// 		body: formData,
// 		headers
// 	};

// 	let response;
// 	try {
// 		response = await fetch(url, options);
// 	} catch (e) {
// 		console.error(e);
// 	} finally {
// 		fs.unlinkSync(oggPath);
// 		fs.unlinkSync(wavPath);
// 	}

// 	if (!response || response.status != 200) {
// 		console.error(response);
// 		return {
// 			text: "",
// 			language: language
// 		};
// 	}

// 	const transcription = await response.json();
// 	return {
// 		text: transcription.text,
// 		language
// 	};
// }

async function convertOggToWav(oggPath: string, wavPath: string): Promise<void> {
	return new Promise((resolve, reject) => {
		ffmpeg(oggPath)
			.toFormat("wav")
			.outputOptions("-acodec pcm_s16le")
			.output(wavPath)
			.on("end", () => resolve())
			.on("error", (err) => reject(err))
			.run();
	});
}