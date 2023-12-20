# LLM (Large Language Models)

Large Language Models (LLMs) como o ChatGPT 3.5 Turbo são modelos avançados de processamento de linguagem natural desenvolvidos pela OpenAI. Esses modelos são treinados em vastos conjuntos de dados e são capazes de realizar uma variedade de tarefas de linguagem, desde responder perguntas até criar conteúdo original. O ChatGPT 3.5 Turbo é uma versão aprimorada que oferece respostas mais rápidas e precisas.

## Uso do ChatGPT 3.5 Turbo no WhatsApp Chatbot

Para o projeto, foi realizado a integração do ChatGPT 3.5 Turbo em uma aplicação, destacando sua utilização para processar e responder a perguntas em tempo real. Abaixo, detalhamos os principais componentes e seu funcionamento:

### Arquitetura de Interação

Abaixo, temos um diagrama que representa a arquitetura de interação do ChatGPT 3.5 Turbo. O ChatGPT 3.5 Turbo é utilizado para processar perguntas e gerar respostas. As respostas são então processadas para extrair informações úteis. Por fim, as informações extraídas são utilizadas para realizar ações, como enviar os pontos de navegação.

![Alt text](../../static/img/intera%C3%A7%C3%A3o-llm.png)

Nesse sentido, o fluxo de interação do usuário inicia com uma mensagem qualquer ao LLM no Whatsapp, que oferecerá uma série de opções de ações. Se a opção escolhida for "solicitar nova peça", conforme o item na extrema direita da figura, o fluxo seguirá segundo as setas acima.

Para visualizar o código completo, acesse o link [clicando aqui](https://www.figma.com/file/R1cXWtvZTLOK0pDjtq5X8u/Diagrama-de-Integra%C3%A7%C3%A3o?type=whiteboard&node-id=0%3A3&t=NZqYKvnWQQvhXTcd-1)

### Variáveis de Ambiente

As variáveis de ambiente são utilizadas para armazenar informações sensíveis, como chaves de acesso a APIs. No projeto, utilizamos as seguintes variáveis de ambiente:

#### Variáveis de API e Modelo ChatGPT

##### `OPENAI_API_KEY`

- **Descrição**: Chave de API para autenticação com os serviços da OpenAI.
- **Valor de Exemplo**: `"sk-DmNDktshq4blsMSFWPssjBlbkFJoMPFyIhZiExg97assewa"`
- **Uso**: Essencial para realizar chamadas autenticadas aos serviços da OpenAI, como a geração de texto e transcrição de voz.

##### `OPENAI_GPT_MODEL`

- **Descrição**: Especifica o modelo GPT da OpenAI a ser utilizado.
- **Valor de Exemplo**: `"gpt-4"`
- **Uso**: Define qual versão do modelo de linguagem (por exemplo, GPT-3, GPT-4) será usada para processamento de linguagem natural.

#### Funcionalidades de Speech-to-Text e Text-to-Speech (TTS)

##### `TRANSCRIPTION_ENABLED`

- **Descrição**: Habilita ou desabilita a funcionalidade de transcrição de voz.
- **Valor de Exemplo**: `true`
- **Uso**: Quando `true`, o sistema irá reconhecer e transcrever mensagens de voz.

##### `TRANSCRIPTION_LANGUAGE`

- **Descrição**: Define o idioma para transcrição de voz.
- **Valor de Exemplo**: `"pt-BR"`
- **Uso**: Informa ao serviço de transcrição qual idioma está sendo falado, possibilitando uma transcrição mais precisa. Dependendo do serviço, pode haver detecção automática de idioma se não especificado.

##### `TRANSCRIPTION_URL`

- **Descrição**: URL do serviço de transcrição de voz da OpenAI.
- **Valor de Exemplo**: `"https://api.openai.com/v1/audio/transcriptions"`
- **Uso**: Endereço para onde as solicitações de transcrição de voz são enviadas.

##### `TTS_URL`

- **Descrição**: URL do serviço de Text-to-Speech (TTS) da OpenAI.
- **Valor de Exemplo**: `"https://api.openai.com/v1/audio/speech"`
- **Uso**: Endereço para onde as solicitações de conversão de texto em fala são enviadas.

##### `TTS_ENABLED`

- **Descrição**: Habilita ou desabilita a funcionalidade de Text-to-Speech.
- **Valor de Exemplo**: `true`
- **Uso**: Quando `true`, o sistema pode converter texto em fala.

#### Conexão de Rede e Banco de Dados

##### `SOCKET_URL`

- **Descrição**: URL do servidor de sockets para comunicação em tempo real.
- **Valores de Exemplo**:
  - `"http://10.128.68.115:3000"`
- **Uso**: Endereços utilizados para estabelecer conexões de socket para comunicação em tempo real entre diferentes partes do sistema.

##### `DATABASE_URL`

- **Descrição**: URL de conexão com o banco de dados PostgreSQL.
- **Valor de Exemplo**: `"postgresql://user:password@host:5432/postgres"`
- **Uso**: Utilizada para conectar o sistema ao banco de dados PostgreSQL, contendo informações de autenticação e localização do banco.

#### Autenticação e Segurança

##### `AUTH_TOKEN`

- **Descrição**: Token utilizado para autenticação de serviços internos.
- **Valor de Exemplo**: `"Chatbot"`
- **Uso**: Usado para garantir que as solicitações entre diferentes serviços do sistema sejam autenticadas e autorizadas.

##### `TOKEN_SECRET`

- **Descrição**: Segredo utilizado para a geração de tokens de autenticação.
- **Valor de Exemplo**: `"Chatbot"`
- **Uso**: Utilizado na criação e validação de tokens, garantindo a segurança na comunicação entre serviços.

Um arquivo `.env` é utilizado para armazenar essas variáveis de ambiente. O arquivo `.env` é ignorado pelo Git, para evitar que as informações sensíveis sejam compartilhadas. Antes de executar o projeto, é necessário criar um arquivo `.env` na raiz do projeto `chatbot/src` e definir as variáveis citadas acima.

### Exemplo de Arquivo .env

Abaixo, temos um exemplo de arquivo `.env` utilizado para configurar o projeto.

```env
OPENAI_API_KEY="sk-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"

OPENAI_GPT_MODEL= "gpt-4"

TRANSCRIPTION_ENABLED=true

PROMPT_OPENAI_POINTS="Responda a pergunta abaixo com base no contexto para encontrar as coordenadas do lugar. Fique atento para possíveis variações no nome quando o usuário perguntar.Sempre responda na língua que o usuário se comunicar. Sempre dê as coordenadas no formato ([x], [y], [z])"

PROMPT_OPENAI_TOOLS="Responda a pergunta abaixo no contexto ao qual você se encontra para encontrar apenas a ferramenta ideal que o usuário está procurando. Fique atento para possíveis variações no nome quando o usuário perguntar. Sempre dê o nome da ferramenta no formato ([nome], [id]) e me diga como posso usar."

TRANSCRIPTION_LANGUAGE="pt-BR"

TRANSCRIPTION_URL="https://api.openai.com/v1/audio/transcriptions"

TTS_URL="https://api.openai.com/v1/audio/speech"

TTS_ENABLED=true

SOCKET_URL="http://10.128.68.115:3000"

AUTH_TOKEN="Chatbot"

TOKEN_SECRET="Chatbot"

DATABASE_URL="postgresql://user:password@host:5432/postgres"

```

### Inicialização da API do OpenAI

```javascript
export function initOpenAI() {
	openai = new OpenAIApi(
		new Configuration({
			apiKey: getConfig("gpt", "apiKey")
		})
	);
}
```

- **OpenAIApi**: Inicializa as configurações necessárias para interagir com o modelo GPT-3.5 Turbo.
- **Configurações**: Define parâmetros como `temperature`, `top_p`, e `max_tokens`, que influenciam o estilo e a extensão das respostas geradas pelo modelo.
- **ApiKey:** Chave de acesso para a API do OpenAI.

### Leitura de Pontos

No contexto do projeto a ideia inicial era que um arquivo `.txt` com os pontos fosse lido e a partir disso fosse feita a extração das coordenadas. Porém, o grupo percebeu que seria mais interessante que o usuário pudesse enviar uma mensagem dizendo onde está a partir de uma lista que o usuário do tipo `admin` conseguisse cadastrar, evitando manutenções desnecessárias no código e a partir disso fosse feita a extração das coordenadas.

- **Teste de Extração de Pontos**

Neste cenário, o usuário está prestes a solicitar um novo item. O chatbot, então, questionará qual é a localização atual do usuário. Em resposta, o usuário indicará o nome do local, escolhendo entre as opções previamente cadastradas no banco de dados. O chatbot, por sua vez, fornecerá as coordenadas geográficas exatas desse local. Estas coordenadas serão extraídas e transmitidas para o servidor através de uma conexão socket. Assim, será possível para o robô navegar até o ponto especificado.

<p align="center" display="flex" width="300">

![Alt text](../../static/img/extra%C3%A7%C3%A3o-pontos.jpg)

</p>

### Processamento da Extração de Pontos

Uma vez inicializada a API, o ChatGPT pode ser utilizado para processar perguntas e gerar respostas. O código abaixo demonstra o processo de processamento de perguntas e geração de respostas utilizado para encontrar as coordenadas de um ponto.

```javascript
export async function getPointOpenAI(message: Message, points) {
	let prompt =  "Responda a pergunta abaixo com base no contexto para encontrar as coordenadas do lugar. Fique atento para possíveis variações no nome quando o usuário perguntar.Sempre responda na língua que o usuário se comunicar. Sempre dê as coordenadas no formato ([x], [y], [z])"
	let jsonPoints = JSON.stringify(points)

	let question = `Lista de pontos: ${jsonPoints}. Pergunta: Identifique a responsta do usuário com base na lista de pontos Resposta: ${message.body} e depois coloque as coordenadas do ponto em formato de float.`


	const response = await openai.createChatCompletion({
		model: 'gpt-4',
    	messages: [{role: 'system', content: prompt}, { role: 'user', content: question } ],

	});

    return response.data.choices[0].message?.content;
}
```

- **Geração do Prompt**: Constrói um prompt baseado na mensagem recebida e na lista de pontos, instruindo o modelo sobre como responder. O prompt é uma string que contém o contexto e a pergunta, e é utilizado para gerar a resposta, no exemplo acima, utilizamos a role `system` para o contexto e `user` para a pergunta.
- **Interpretação de Resposta**: Envia o prompt para o ChatGPT 3.5 Turbo e recebe a resposta, que é processada para extrair informações úteis.

### Extração de Informações da Resposta

Para a extração de informações da resposta, utilizamos uma expressão regular para extrair as coordenadas do formato ([x], [y], [z]) da resposta. O código abaixo demonstra o processo de extração de informações utilizado para encontrar as coordenadas de um ponto.

```javascript
let pointResponse = response.data.choices[0].message?.content;

	const regex: RegExp = /-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?,\s*-?\d+(?:\.\d+)?/gi;
    const match = pointResponse?.match(regex);
	const socket = io("http://10.128.64.39:3000");
    if (match) {
		match.forEach(coordinateString => {
			// Splitting the matched string into individual numbers
			const parts = coordinateString.split(',').map(part => parseFloat(part.trim()));
			const [x, y, z] = parts;
			socket.emit("send_points", {x, y, z});
			message.reply(pointResponse as any);

		});
    }
	else{
		message.reply("Não consegui encontrar o ponto. Tente novamente.")
	}
```

- **Extração de Coordenadas**: Utiliza uma expressão regular para extrair coordenadas do formato ([x], [y], [z]) da resposta.
- **Comunicação com Socket**: Envia as coordenadas extraídas para um servidor via socket, indicando uma aplicação em tempo real.
- **Feedback ao Usuário**: Responde ao usuário com a informação extraída ou uma mensagem de erro caso as coordenadas não sejam encontradas.


## Considerações de Implementação do ChatGPT 3.5 Turbo

1. **Precisão do Modelo**: Garantir que o modelo esteja bem treinado e capaz de interpretar e responder a uma ampla variedade de consultas com precisão.
2. **Segurança e Privacidade**: Tomar medidas para proteger dados sensíveis que possam ser processados pelo modelo.
3. **Gerenciamento de Erros**: Implementar um robusto sistema de tratamento de erros para lidar com respostas inesperadas ou erros de processamento.

## Conclusão

O ChatGPT 3.5 Turbo representa um avanço significativo na área de LLMs, oferecendo respostas rápidas e precisas para uma variedade de tarefas de processamento de linguagem natural. A integração dessa tecnologia em aplicações, como demonstrado no código fornecido, pode melhorar significativamente a interação usuário-máquina e a eficiência do processamento de dados. É fundamental, no entanto, garantir a precisão, segurança e privacidade ao trabalhar com esses modelos avançados.
