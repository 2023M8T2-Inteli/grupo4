# Github Actions e Esteira de Deploy

O Github Actions é uma ferramenta de automação de tarefas que pode ser utilizada para diversas finalidades, como por exemplo, a criação de uma esteira de deploy. Esta ferramenta é integrada ao Github, e permite a criação de workflows que são executados quando determinados eventos ocorrem, como por exemplo, a criação de uma nova `tag` em um repositório ou a criação de um novo `pull request` a partir de um `commit` em uma `branch` específica. Facilitando o processo de desenvolvimento e deploy de aplicações. Neste documento, será apresentado como configurar o Github Actions para realizar o deploy de uma aplicação web em um servidor remoto AWS. 

## Esteira do Chatbot - WhatsApp

#### Visão Geral
Este documento descreve o processo automatizado de teste, construção, e implementação (deploy) de um chatbot do WhatsApp usando GitHub Actions. O pipeline é dividido em três partes principais: Teste (`TEST_CHATBOT`), Integração Contínua (`CI_CHATBOT`), e Entrega Contínua (`CD_CHATBOT`).

#### Vantagens do Uso do GitHub Actions
- **Automação Completa**: Desde testes até deploy, todo o processo é automatizado.
- **Integração Fácil com AWS**: Integração com ECR e EC2 para deploy.
- **Flexibilidade**: Possibilidade de customização e expansão conforme as necessidades do projeto.
- **Segurança**: Uso de variáveis de ambiente secretas para proteger informações sensíveis.

#### Processo de Execução
1. **Teste (`TEST_CHATBOT`)**:
   - Ativação: Quando há um push na branch `main` ou por meio de `workflow_dispatch`.
   - Funções: Checa o código, cria um arquivo `.env` com variáveis de ambiente, constrói e limpa uma imagem Docker.

2. **Integração Contínua (`CI_CHATBOT`)**:
   - Dependência: Necessita da conclusão bem-sucedida do `TEST_CHATBOT`.
   - Funções: Configura credenciais AWS, faz login no Amazon ECR, constrói, marca, e envia a imagem para o ECR.

3. **Entrega Contínua (`CD_CHATBOT`)**:
   - Dependência: Necessita da conclusão bem-sucedida do `CI_CHATBOT`.
   - Funções: Configura credenciais AWS, copia o arquivo `.env` para a instância EC2, e executa scripts no servidor EC2 para rodar o chatbot.

#### Variáveis de Ambiente Utilizadas
- Variáveis de Configuração do OpenAI e Serviços Relacionados:
  - `OPENAI_API_KEY`
  - `OPENAI_GPT_MODEL`
  - `TRANSCRIPTION_ENABLED`
  - `PROMPT_OPENAI_POINTS`
  - `PROMPT_OPENAI_TOOLS`
  - `TRANSCRIPTION_LANGUAGE`
  - `TRANSCRIPTION_URL`
  - `TTS_URL`
  - `TTS_ENABLED`
- Variáveis de Conexão e Configuração de Rede:
  - `SOCKET_URL`
  - `DATABASE_URL`
- Variáveis de Autenticação e Configuração AWS:
  - `AWS_ACCESS_KEY_ID`
  - `AWS_SECRET_ACCESS_KEY`
  - `AWS_SESSION_TOKEN`
  - `AWS_REGION`
- Variáveis de Configuração do Projeto Google Cloud:
  - `PRIVATE_KEY`
  - `TYPE`
  - `PROJECT_ID`
  - `PRIVATE_KEY_ID`
  - `CLIENT_EMAIL`
  - `CLIENT_ID`
  - `AUTH_URI`
  - `TOKEN_URI`
  - `AUTH_PROVIDER_X509_CERT_URL`
  - `CLIENT_X509_CERT_URL`
  - `UNIVERSE_DOMAIN`

#### Notas Adicionais
- **Segurança**: É crucial manter as variáveis de ambiente secretas e seguras, especialmente as chaves de API e credenciais de acesso.
- **Customização**: Dependendo das necessidades específicas do projeto, este pipeline pode ser modificado ou expandido.
- **Monitoramento e Manutenção**: É recomendável monitorar a execução das actions e realizar manutenção regular para garantir a eficiência e segurança do pipeline.