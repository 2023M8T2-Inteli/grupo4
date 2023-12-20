# TTS e STT no Whatsapp

## Speech to Text (STT)

A construção do Speech to Text (STT) foi feita utilizando a API da OpenAI utilazando o whisper mode, que é um modo de reconhecimento de voz que permite que o usuário fale em voz baixa e o modelo ainda seja capaz de transcrever a fala. Para isso, foi necessário criar uma função que recebe o áudio e o transforma em texto. O código abaixo demonstra o processo de transformação de áudio em texto.


```javascript
export async function transcribeOpenAI(
  audioBuffer: Buffer
): Promise<{ text: string; language: string }> {
  const url =
    process.env.TRANSCRIPTION_URL ||
    'https://api.openai.com/v1/audio/transcriptions';
  let language = process.env.TRANSCRIPTION_LANGUAGE || 'pt-BR';
  const oggPath = path.join(__dirname + '/media/', randomUUID() + '.ogg');
  const wavFilename = randomUUID() + '.wav';
  const wavPath = path.join(__dirname + '/media/', wavFilename);
  fs.writeFileSync(oggPath, audioBuffer);
  try {
    await convertOggToWav(oggPath, wavPath);
  } catch (e) {
    fs.unlinkSync(oggPath);
    return {
      text: '',
      language,
    };
  }

  const formData = new FormData();

  formData.append('file', audioBuffer, {
    filename: wavFilename,
    contentType: 'audio/wav',
  });
  formData.append('model', 'whisper-1');
  formData.append('response_format', 'json');
  let response;
  try {
    // response = await fetch(url, options);
    response = await axios.post(url, formData, {
      headers: {
        ...formData.getHeaders(),
        Authorization: `Bearer ${process.env.OPENAI_API_KEY}`,
      },
    });
  } catch (e) {
    console.error(e);
    return {
      text: '',
      language: language,
    };
  } finally {
    fs.unlinkSync(oggPath);
    fs.unlinkSync(wavPath);
    const transcription = await response.data;

    return {
      text: transcription.text,
      language,
    };
  }
}

async function convertOggToWav(
  oggPath: string,
  wavPath: string
): Promise<void> {
  return new Promise((resolve, reject) => {
    ffmpeg(oggPath)
      .toFormat('wav')
      .outputOptions('-acodec pcm_s16le')
      .output(wavPath)
      .on('end', () => resolve())
      .on('error', (err) => reject(err))
      .run();
  });
}

```

Na função acima, o áudio B é convertido de ogg para wav, pois a API da OpenAI só aceita arquivos no formato wav. e entre outros formatos que podem ser consultados na [documentação da OpenAI](https://platform.openai.com/docs/guides/speech-to-text). Após a conversão, o áudio é enviado para a API da OpenAI e o retorno é o texto transcrito. **Nota:** Em Typescript é necessário realizar as chamadas para a API da OpenAI utilizando o axios, pois ainda não é suportado a biblioteca da OpenAI.

## Text to Speech (TTS)

A construção do Text to Speech (TTS) foi feita utilizando a API da OpenAI. Para isso, foi necessário criar uma função que recebe o texto e o transforma em áudio enviando para a API da OpenAI, ao qual retornar um `arraybuffer` e a partir disso é possível transformar em um arquivo de áudio, que é enviado para o usuário na forma de mensagem de voz pelo Whatsapp. Para isso, foi necessário criar uma função que recebe o texto e o transforma em áudio, além das informações de idioma e velocidade. O código abaixo demonstra o processo de transformação de texto em áudio.

```javascript

export async function speechOpenAI(
  message: Message,
  client: Client,
  text: string | undefined
): Promise<String> {
  const user = await userService.getUser(message.from);
  const url = process.env.TTS_URL || 'https://api.openai.com/v1/audio/speech';
  let response;
  try {
    response = await axios.post(
      url,
      {
        model: 'tts-1',
        voice: user?.voice || 'alloy',
        velocity: user?.speedVoice || 0.87,
        input: text,
      },
      {
        headers: {
          Authorization: `Bearer ${process.env.OPENAI_API_KEY}`,
          'Content-Type': 'application/json',
        },
        responseType: 'arraybuffer',
      }
    );
  } catch (e) {
    console.error(e);
    return '';
  } finally {
    const audioBase64 = Buffer.from(response.data, 'binary').toString('base64');
    const messageMedia = new MessageMedia('audio/ogg', audioBase64);
    await client.sendMessage(message.from, messageMedia, {
      sendAudioAsVoice: true,
    });
    return audioBase64;
  }
}
```
**Nota:** Em Typescript é necessário realizar as chamadas para a API da OpenAI utilizando o axios, pois ainda não é suportado a biblioteca da OpenAI.

## Teste de Integração STT e TTS

Durante a Sprint 4, foi realizado um teste de integração entre o STT e o TTS. O teste consistiu em realizar o pedido de uma peça por meio do envio uma mensagem de voz para o chatbot, que transcreveu a mensagem e enviou uma resposta em áudio de volta. O teste foi realizado com sucesso, como pode ser visto no vídeo abaixo.

<iframe width="560" height="315" src="https://youtube.com/embed/PD3DZXWczUg" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen> </iframe>