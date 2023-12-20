# Chatbot no Whatsapp

Como interface de interação com o usuário, optou-se pelo WhatsApp, devido à sua ampla acessibilidade e popularidade, uma vez que é um aplicativo já instalado nos celulares da maioria dos usuários. No Brasil, 99% da população utiliza o aplicativo. O WhatsApp também facilita a criação de chatbots, que são sistemas automatizados capazes de interagir com o usuário por meio de mensagens de texto, voz, imagens, vídeos, documentos, localização e outros tipos de mídia. Esta integração beneficia os funcionários da Ambev, que não precisam se adaptar a uma nova ferramenta, já que o WhatsApp faz parte do seu cotidiano.

Por meio desta integração, os usuários podem interagir com o chatbot enviando perguntas e recebendo respostas via mensagens de texto. O chatbot é projetado para interpretar as mensagens enviadas, identificar a intenção por trás delas e fornecer a resposta mais adequada.


## Integração com o WhatsApp

A integração com o WhatsApp é realizada por meio de uma árvore de interação e decisão, desenvolvida na linguagem de programação TypeScript. Esta abordagem permite uma programação mais robusta da aplicação. Todo o processo é construído com o auxílio da biblioteca whatsapp-web.js, que simula um cliente do WhatsApp Web. Isso possibilita a troca de mensagens entre o chatbot e o usuário.

## Árvore de Interação

Atualemente a árvore de interação do chatbot é possuir 4 funcionalidades rodando, sendo eles:

1. Verificação de acesso: O chatbot verifica se o usuário possui acesso ao sistema, caso não possua, o chatbot solicita que o usuário fale com um atendente.
2. Acompanhamento de pedido: O chatbot verifica se o usuário deseja acompanhar o status de um pedido, caso sim, o chatbot solicita o número do pedido e retorna o status do pedido.
3. Cancelamento de pedido: O chatbot verifica se o usuário deseja cancelar um pedido, caso sim, o chatbot solicita o número do pedido e cancela o pedido.
4. Falar com um atendente: O chatbot verifica se o usuário deseja falar com um atendente, caso sim, o chatbot envia o contato de um atendente para o usuário entrar em contato.
5. Alteração no nome do usuário: O chatbot verifica se o usuário deseja alterar o nome cadastrado no sistema, caso sim, o chatbot solicita o novo nome do usuário e altera o nome cadastrado no sistema.

## Arquitetura do Chatbot

O diagrama apresentado ilustra a arquitetura e o fluxo de processos de um sistema de chatbot integrado com um TurtleBot 3. A arquitetura é dividida em quatro principais componentes: Turtle Bot 3, Bridge (Ponte), Cloud (Nuvem) e WhatsApp. Aqui está uma descrição detalhada de cada componente e seu funcionamento no sistema:

<p align="center" display="flex" width="300">

![Alt text](../../static/img/diagrama-chatbot.png)

</p>

Para visualizar o código completo, acesse o link [clicando aqui](https://www.figma.com/file/EyWTXX5oJpVthHioS1puEQ/Diagrama-Chatbot?type=whiteboard&node-id=1%3A2&t=kOvZbGWgMXaA92c9-1)

### Turtle Bot 3

- **NAV2**: Representa o componente de navegação do TurtleBot 3. O NAV2 é responsável por orientar o robô em seu ambiente, utilizando mapas e sensores para navegar até destinos pré-determinados.
- O TurtleBot 3 recebe instruções para enviar os robôs disponíveis para os destinos especificados.

### Bridge (Ponte)

- **Fila de locais de destino**: Armazena as coordenadas ou identificadores dos destinos que o TurtleBot 3 deve visitar. Essa fila é usada para gerenciar e direcionar o robô para vários pontos conforme necessário.

### Cloud (Nuvem)

- **Backend**: Núcleo lógico do sistema onde ocorrem o processamento de dados e as decisões.
- **API**: Interface de programação de aplicativos que facilita a comunicação entre o chatbot e o backend. Ela adiciona solicitações à fila de mensagens e executa funções assíncronas.
- **Funções Assíncronas**: Operações que são realizadas em paralelo com outras tarefas, sem bloquear a execução do sistema.
- **Fila de Mensagens**: Buffer que armazena as mensagens do usuário antes de serem processadas pelo sistema.
- **Organizador de acordo com o volume de pedidos**: Um sistema que prioriza e organiza os pedidos com base no volume de solicitações recebidas.
- **Fila de pontos**: Lista de destinos ou coordenadas que serão enviadas para o TurtleBot 3.
- **TTS & STT (Text-To-Speech & Speech-To-Text)**: Componentes responsáveis por converter texto em fala e vice-versa, permitindo uma interação mais natural com o usuário.
- **LLM (Large Language Model)**: Modelo de linguagem avançado que processa e gera respostas para as mensagens dos usuários.

### Banco de Dados

- **Tabela de usuários**: Armazena informações dos usuários, como identificação e permissões de acesso.

### WhatsApp

- **Usuário**: Pessoa que interage com o chatbot via WhatsApp.
- **ChatBot**: Aplicação que recebe mensagens do usuário e responde automaticamente, utilizando as capacidades do LLM para gerar respostas relevantes.

## Fluxo Geral

1. O **usuário do WhatsApp** envia uma mensagem para o **ChatBot**.
2. O **ChatBot**, através de sua integração com o **LLM**, processa a mensagem e determina a ação necessária.
3. As ações podem incluir o envio de comandos para o **TurtleBot 3** (via **NAV2**) ou a execução de operações no **backend**.
4. O **backend** executa **funções assíncronas**, que podem envolver consultas à **tabela de usuários** no **banco de dados** e adicionar solicitações à **fila de mensagens**.
5. Com base no processamento realizado na **nuvem**, os robôs são direcionados pela **ponte** para os locais de destino armazenados na **fila de locais de destino**.

## Features da Sprint 3

Durante a sprint 3, o grupo desenvolveu novas funcionalidades para o chatbot, que incluem:

- **Camada de Segurança:** O chatbot verifica se o usuário tem acesso ao sistema. Caso não tenha, solicita que o usuário entre em contato com um atendente.
- **Novas Funcionalidades para o Usuário Comum:** Incluem acompanhamento e cancelamento de pedido, interação com um atendente e alteração no nome do usuário, além do início do desenvolvimento da funcionalidade de solicitação de peças.
- **Módulo de Administração:** Permite que o administrador do sistema cadastre pontos e conceda permissões aos usuários.
- **Integração com o Turtlebot3:** Possibilita que o chatbot envie comandos para o robô se movimentar dentro da cervejaria.

Essas funcionalidades foram desenvolvidas com o objetivo de melhorar a experiência do usuário com o chatbot, que visa ser a interface principal de interação entre o usuário e o robô, bem como entre o usuário e o sistema.

### Detalhamento dos Tipos de Usuários e Interações

#### Módulo LEAD (Cadastro)

Este módulo descreve o processo pelo qual um novo usuário inicia a interação com o chatbot e se registra no sistema.

- **Fluxo de Cadastro**

1. **Iniciação**: O usuário inicia a conversa com uma saudação, como "Oi".
2. **Solicitação de Cadastro**: O chatbot pede informações essenciais, como o nome completo, para criar uma nova conta no sistema.
3. **Criação de Conta**: Após receber as informações necessárias, o chatbot registra o usuário no banco de dados e informa o acesso restrito ao sistema.

- **Comandos e Respostas**
- **`Oi`**: Inicia a interação com o chatbot.
- **`Fornece Nome Completo`**: Usuário fornece o nome completo para o cadastro.
- **Exemplo de Interação**

<p align="center" display="flex" width="300">

![Alt text](../../static/img/lead.png)

</p>

#### Módulo User (Usuário Comum)

Este módulo detalha as interações disponíveis para usuários já registrados no sistema.

- **Fluxo de Interação**

1. **Menu de Serviços**: O chatbot oferece um menu de serviços, que pode incluir opções como solicitar novas peças, acompanhar o status do pedido, entre outras.
2. **Seleção de Serviços**: O usuário seleciona um serviço digitando o número correspondente.
3. **Processamento**: O chatbot processa a solicitação e realiza a ação necessária, fornecendo feedback ao usuário.

- **Comandos e Respostas**
- **`Número correspondente ao serviço`**: O usuário digita o número para selecionar o serviço desejado.
- **`Feedback`**: O chatbot responde com o status da ação solicitada ou informações adicionais.
- **Exemplo de Interação**

<p align="center" display="flex" width="300">

![Alt text](../../static/img/user.png)

</p>

#### Módulo Admin (Administrador)

Este módulo é dedicado aos usuários com privilégios administrativos, permitindo-lhes gerenciar aspectos do chatbot e do sistema subjacente.

- **Fluxo de Administração**

1. **Menu de Administração**: O chatbot exibe um menu de administração com opções como cadastrar novos pontos e autorizar acessos.
2. **Execução de Comandos de Administração**: O administrador executa comandos específicos para gerenciar o sistema.
3. **Gerenciamento de Usuários e Serviços**: O administrador tem a capacidade de adicionar, modificar ou remover usuários e serviços do sistema.

- **Comandos e Respostas**
- **`Cadastrar Novo Ponto`**: Permite ao administrador adicionar um novo local ao sistema.
- **`Autorizar Acesso`**: Concede ou revoga acesso aos usuários para determinados serviços ou funcionalidades.
- **Exemplo de Interação**

<p align="center" display="flex" width="300">

![Alt text](../../static/img/admin.png)

</p>

## Configuração e Execução do Chatbot

Para rodar o ChatBot, siga as instruções abaixo com atenção. Estes passos são essenciais para iniciar a aplicação de forma correta e garantir seu funcionamento adequado.

### Pré-Requisitos

Antes de começar, confirme se você tem a última versão do Node.js instalada. Isso é crucial para a execução do projeto. Você pode verificar a versão instalada com o comando:

```sh
node --version
```

Se necessário, atualize ou instale o Node.js através do [site oficial](https://nodejs.org/).

### Instalação das Dependências

Com o Node.js instalado, navegue até o diretório do projeto `chatbot/src` e instale as dependências do projeto com o gerenciador de pacotes NPM:

```sh
cd src/chatbot/src
npm install
```

Esse comando irá baixar e instalar todas as bibliotecas necessárias definidas no arquivo `package.json`.

### Configuração de Variáveis de Ambiente

As variáveis de ambiente são cruciais para a configuração da aplicação. Crie um arquivo `.env` no diretório `chatbot/src` e insira as chaves e valores apropriados como no exemplo abaixo:

```env
# Chave de API da OpenAI
OPENAI_API_KEY="sk-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"

# Modelo GPT a ser utilizado (padrão: gpt-3.5-turbo)
OPENAI_GPT_MODEL="gpt-3.5-turbo"

# Máximo de tokens por solicitação ao GPT
MAX_MODEL_TOKENS=2000

# Habilita a transcrição de mensagens de voz
TRANSCRIPTION_ENABLED=true

# Idioma da transcrição (caso o serviço de transcrição não detecte automaticamente)
TRANSCRIPTION_LANGUAGE="pt-BR"

# Habilita a conversão de texto em fala
TTS_ENABLED=true

# Token de autenticação para o chatbot
AUTH_TOKEN="Chatbot"

# Segredo para o token de autenticação
TOKEN_SECRET="Chatbot"

# URL de conexão com o banco de dados
DATABASE_URL="postgresql://user:password@host:5432/postgres"
```

Substitua os valores de exemplo pelas chaves e informações reais que você possui.

### Iniciando o ChatBot

Com as variáveis de ambiente configuradas, você está pronto para iniciar o chatbot. Execute o seguinte comando:

```sh
npm run start
```

Aguarde o inicialização e fique atento ao terminal para o próximo passo.

### Conectando ao WhatsApp

Após iniciar o chatbot, um QR Code será exibido no terminal. Use o aplicativo WhatsApp no seu celular para escanear esse código:

1. Abra o WhatsApp.
2. Vá até 'Configurações' > 'WhatsApp Web/Desktop'.
3. Escaneie o QR Code mostrado no terminal.

Com esses passos, o chatbot estará ativo e pronto para interagir através do WhatsApp conectado.

Lembre-se de monitorar o terminal para quaisquer mensagens de erro ou logs importantes que possam ajudar a diagnosticar problemas caso o chatbot não funcione como esperado.

## Conclusão

Entendemos que o chatbot desempenhará um papel fundamental no projeto desenvolvido, atuando como interface de interação entre o usuário e o robô. Por meio do chatbot, o usuário poderá solicitar a movimentação de peças, acompanhar o status de pedidos, cancelar pedidos e entrar em contato com um atendente. Adicionalmente, o chatbot será responsável por enviar notificações ao usuário, como, por exemplo, informar quando o robô estiver a caminho do almoxarifado para entregar uma peça.

Este processo de desenvolvimento é extenso, e a equipe tem se dedicado a ele desde a sprint 1, com especial atenção a essa parte do projeto, principalmente após a visita à Cervejaria Ambev em Guarulhos. Essa experiência permitiu uma compreensão mais aprofundada do processo de movimentação de peças e de como o chatbot pode ser útil nesse contexto. Reconhecemos que esta tecnologia é a única em que o almoxarife confia plenamente, e a facilidade de uso pode contribuir significativamente para a eficiência do trabalho dessa persona e dos demais funcionários da Ambev.
