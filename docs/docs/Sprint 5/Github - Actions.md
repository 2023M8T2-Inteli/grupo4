# Github Actions e Esteira de Deploy

O Github Actions é uma ferramenta de automação de tarefas que pode ser utilizada para diversas finalidades, como por exemplo, a criação de uma esteira de deploy. Esta ferramenta é integrada ao Github, e permite a criação de workflows que são executados quando determinados eventos ocorrem, como por exemplo, a criação de uma nova `tag` em um repositório ou a criação de um novo `pull request` a partir de um `commit` em uma `branch` específica. Facilitando o processo de desenvolvimento e deploy de aplicações. Neste documento, será apresentado como configurar o Github Actions para realizar o deploy de uma aplicação web em um servidor remoto AWS.

## Vantagens do Uso do GitHub Actions

- **Automação Completa**: Desde testes até deploy, todo o processo é automatizado.
- **Integração Fácil com AWS**: Integração com ECR e EC2 para deploy.
- **Flexibilidade**: Possibilidade de customização e expansão conforme as necessidades do projeto.
- **Segurança**: Uso de variáveis de ambiente secretas para proteger informações sensíveis.
- 

## Esteira do Chatbot - WhatsApp

O processo é automatizado por meio de teste, construção, e implementação (deploy) do chatbot com WhatsApp usando GitHub Actions. O pipeline é dividido em três partes principais: Teste (`TEST_CHATBOT`), Integração Contínua (`CI_CHATBOT`), e Entrega Contínua (`CD_CHATBOT`).

### Processo de Execução

1. **Teste (`TEST_CHATBOT`)**:

   - Ativação: Quando há um push na branch `main` ou por meio de `workflow_dispatch`.
   - Funções: Checa o código, cria um arquivo `.env` com variáveis de ambiente, constrói e limpa uma imagem Docker.
2. **Integração Contínua (`CI_CHATBOT`)**:

   - Dependência: Necessita da conclusão bem-sucedida do `TEST_CHATBOT`.
   - Funções: Configura credenciais AWS, faz login no Amazon ECR, constrói, marca, e envia a imagem para o ECR.
3. **Entrega Contínua (`CD_CHATBOT`)**:

   - Dependência: Necessita da conclusão bem-sucedida do `CI_CHATBOT`.
   - Funções: Configura credenciais AWS, copia o arquivo `.env` para a instância EC2, e executa scripts no servidor EC2 para rodar o chatbot.

### Variáveis de Ambiente Utilizadas

- Variáveis de Configuração do OpenAI e Serviços Relacionados:
  - `OPENAI_API_KEY`
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

## Esteira da Interface

Este processo foi configurado para a interface, incluindo testes, construção, e deploy. O workflow é dividido em três fases principais: Teste (`TEST_INTERFACE`), Integração Contínua (`CI_INTERFACE`), e Entrega Contínua (`CD_INTERFACE`).

#### Processo de Execução

1. **Teste (`TEST_INTERFACE`)**:

   - Ativado por um push na branch `main` ou por `workflow_dispatch`.
   - Cria arquivos `.env` para os diretórios backend, admin e robot.
   - Configura Docker Compose e testa containers individuais.
2. **Integração Contínua (`CI_INTERFACE`)**:

   - Dependente da conclusão bem-sucedida do `TEST_INTERFACE`.
   - Checa o repositório e configura credenciais AWS.
   - Constrói, marca e envia imagens Docker para o Amazon ECR para backend, admin e robot.
3. **Entrega Contínua (`CD_INTERFACE`)**:

   - Dependente da conclusão bem-sucedida do `CI_INTERFACE`.
   - Configura credenciais AWS e copia o arquivo `.env` para uma instância EC2.
   - Executa comandos em uma instância EC2 para iniciar a interface.

#### Variáveis de Ambiente Utilizadas

- **Backend**:
  - `OPENAI_API_KEY`
  - `DATABASE_URL`
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
- **Admin e Robot**:
  - `NEXT_PUBLIC_HOST`
  - `NEXT_PUBLIC_SOCKET`
  - `NEXT_PUBLIC_BACKEND`
- **AWS**:
  - `AWS_ACCESS_KEY_ID`
  - `AWS_SECRET_ACCESS_KEY`
  - `AWS_SESSION_TOKEN`
  - `AWS_REGION`

## Esteira do Terraform - Up Infrastructure

A esteira do terraform foi configurado para provisionar e gerenciar infraestrutura usando Terraform. O workflow é acionado por alterações na pasta 'infrastructure' na branch `main` ou por meio de `workflow_dispatch`.

### Processo de Execução

1. **Checkout**: Obtém o código mais recente da branch `main`.
2. **Setup do Terraform**: Configura o Terraform usando um token de API armazenado em segredo.
3. **Inicialização do Terraform**: Executa `terraform init` na pasta 'infrastructure'.
4. **Checagem de Mudanças**: Verifica se houve mudanças na pasta 'infrastructure' desde o último commit. Se não houver mudanças, o workflow é encerrado.
5. **Execução do Terraform Plan e Apply**: Se mudanças forem detectadas, executa `terraform plan` e `terraform apply` para aplicar as alterações de infraestrutura.

### Variáveis de Ambiente e Segredos Utilizados

- `TF_API_TOKEN`: Token de API para autenticação com o provedor do Terraform. Armazenado como um segredo no GitHub Actions.

## Notas Adicionais

- **Manutenção e Monitoramento**: É importante monitorar regularmente as execuções do workflow para garantir que tudo funcione conforme esperado.
- **Segurança das Variáveis de Ambiente**: As variáveis de ambiente, especialmente as que contêm credenciais e chaves, devem ser mantidas em segredo e não expostas publicamente.
- **Customização do Workflow**: Este workflow pode ser modificado para se adequar a mudanças nos requisitos do projeto ou para adicionar novas funcionalidades ao processo de CI/CD.
- **Documentação do Código**: Manter a documentação do código atualizada é essencial para garantir que as mudanças no pipeline sejam bem compreendidas e fáceis de manter.

## Considerações Finais

Os fluxos de trabalho de GitHub Actions descritos anteriormente para o Chatbot WhatsApp, a Interface e a Infraestrutura via Terraform ilustram a eficácia da automatização de processos críticos em desenvolvimento de software e gerenciamento de infraestrutura.

**Justificativas para a Utilização de GitHub Actions:**

1. **Eficiência e Consistência**: A automação com GitHub Actions garante uma execução consistente e eficiente de tarefas complexas. Ela elimina a possibilidade de erros humanos em processos repetitivos, aumentando a confiabilidade das operações.
2. **Integração e Flexibilidade**: A capacidade de integrar perfeitamente com uma variedade de ferramentas e serviços, como AWS e Terraform, torna o GitHub Actions uma solução versátil para muitas necessidades de CI/CD e gerenciamento de infraestrutura.
3. **Segurança Reforçada**: O uso de variáveis de ambiente secretas e permissões restritas nos workflows de GitHub Actions assegura a proteção de informações sensíveis e infraestrutura crítica.
4. **Adaptação às Mudanças**: A capacidade de ajustar rapidamente os workflows para acomodar novos requisitos ou mudanças no projeto é uma vantagem significativa, oferecendo agilidade ao desenvolvimento e operações.
5. **Monitoramento e Manutenção Simplificados**: Com GitHub Actions, o monitoramento e a manutenção dos processos de CI/CD e infraestrutura tornam-se mais gerenciáveis, graças à automação e registros detalhados das ações executadas.

## Conclusão

A utilização do GitHub Actions para automatizar o processo de teste, construção, e deploy do Chatbot WhatsApp, a interface de usuário, e a infraestrutura via Terraform, não só otimiza significativamente o fluxo de trabalho de desenvolvimento, mas também reforça a segurança, a consistência, e a eficiência operacional. A habilidade de customizar e expandir esses workflows conforme as necessidades evoluem faz do GitHub Actions uma escolha robusta e escalável para automação em projetos de software e infraestrutura.
