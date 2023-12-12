# Backend de interfaces

Ambas as interfaces consomem uma API em Node.js que se conecta com o Google Cloud para Text-to-Speech (TTS) e Speech-to-Text (STT), OpenAI para conversas contextualizadas de chatbot, e um banco de dados na AWS para manter o estado atual do sistema.

O diretório está estruturado em duas partes: serviços e rotas. Os serviços centralizam as integrações com APIs externas, como a conexão com OpenAI e as funções de transcrição e geração de áudio com o Google Cloud. Já as rotas definem os endpoints possíveis, tanto para operações CRUD em tabelas quanto para intermediar a comunicação por áudio com o chat GPT contextualizado.

## Como rodar
### Pré-Requisitos

Certifique-se de ter o Node.js instalado em seu ambiente. Caso ainda não o tenha, você pode fazer o download em [Node.js website](https://nodejs.org/).

### Credenciais do Google Cloud

Baixe o arquivo de autenticação JSON do Google Cloud para a aplicação. Guarde esse arquivo em um local seguro.

No arquivo `.env` do diretório `src/express-back`, adicione a seguinte linha, especificando o caminho do arquivo de autenticação:

```bash
    GOOGLE_APPLICATION_CREDENTIALS=/caminho/para/seu/arquivo-de-autenticacao.json
```

### Chave de API da OpenAI

Para obter a chave de API da OpenAI, visite o [site da OpenAI](https://beta.openai.com/signup/) e registre-se para obter uma chave de API. Após obtê-la, adicione a seguinte linha no arquivo `.env` do diretório `src/express-back`, substituindo `SUA_CHAVE_DE_API` pela chave fornecida pela OpenAI:

```bash
OPENAI_API_KEY=SUA_CHAVE_DE_API
```

### Configuração do Banco de Dados

A Interface de Administração se conecta com um servidor e requer a configuração do URI do banco de dados no arquivo `.env`, no diretório `src/express-back`:

```bash
# No arquivo .env
DATABASE_URL=sua-url-do-banco-de-dados
```

### Instalação de Dependências

No diretório `src/express-back` execute o seguinte comando para instalar as dependências necessárias:

```bash
npm i
```

### Execução do Sistema
Após configurar as credenciais e instalar as dependências, inicie o backend com o seguinte comando:

```bash
npm run dev
```
Certifique-se de manter o ambiente configurado corretamente para garantir uma execução sem problemas.

## Conclusão

Em conclusão, o backend de interfaces emerge como o componente essencial que possibilita a integração harmoniosa entre as interfaces do sistema e os serviços externos fundamentais. A arquitetura modularizada, dividida entre serviços e rotas, proporciona uma organização eficiente para lidar com as diversas funcionalidades necessárias para o funcionamento do sistema.