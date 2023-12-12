# Logs de Dados

Logs são registros de eventos que acontecem dentro de um software ou sistema. Eles são fundamentais para o monitoramento, depuração e rastreamento de atividades, fornecendo informações detalhadas sobre o comportamento e o estado do sistema.

## A Importância Crucial dos Logs

Logs são fundamentais para uma variedade de funções essenciais no ciclo de vida de qualquer aplicação e sistema, atuando como registros detalhados de eventos. Eles são indispensáveis para:

- **Depuração Eficiente**: Facilitam a identificação e resolução de bugs, fornecendo insights detalhados sobre o comportamento do sistema no momento da ocorrência de uma falha.

- **Monitoramento Contínuo**: São cruciais para acompanhar o desempenho e a saúde do sistema, permitindo a detecção precoce de condições anormais ou degradantes.

- **Auditoria de Segurança Rigorosa**: Desempenham um papel vital na segurança, registrando acessos e alterações no sistema, o que ajuda a detectar atividades suspeitas e a garantir a conformidade com normas de segurança.

- **Análise de Dados Abrangente**: Oferecem um vasto reservatório de informações para análises estatísticas e insights de negócios, contribuindo para a tomada de decisões baseada em dados e o aprimoramento contínuo das operações.

## Tipos Diversificados de Logs

Existem diversos tipos de logs, cada um servindo a um propósito específico e capturando diferentes aspectos do funcionamento de sistemas e aplicações:

- **Logs de Aplicação**: Essenciais para entender o comportamento interno da aplicação, estes logs registram eventos que ocorrem durante a execução da aplicação, como transações, operações de usuários e processos internos.

- **Logs de Sistema**: Focam no nível do sistema operacional, monitorando eventos críticos que afetam a infraestrutura subjacente, como inicializações de sistema, falhas de hardware e alertas de segurança.

- **Logs de Acesso**: São vitais para auditar e analisar o tráfego de rede e solicitações recebidas por servidores, como solicitações HTTP, acessos a APIs e atividades de autenticação.

- **Logs de Erro**: Capturam erros, exceções e falhas de sistema, fornecendo detalhes cruciais para investigações de problemas e para a implementação de correções e melhorias no sistema.

## Tipos de Logs no Projeto

Para garantir um gerenciamento eficiente e uma análise abrangente de nosso sistema, o sistema de logs foi estrategicamente dividido em duas partes principais:

- [Dedicada ao registro de atividades no chatbot](#logs-de-erro-no-chatbot)
- [Dedicada no registro das operações no sistema que engloba o robô](#logs-das-interações-dos-nós-ros)

Essa separação foi crucial para otimizar o monitoramento e a manutenção de cada componente do sistema.

### Logs de Erro no Chatbot

Os logs de erro no chatbot são acionados por meio da utilização do `try` e `catch`, onde o `catch` é responsavel por mandar algum tratamento do erro ocorrido. No chatbot, os logs de erros são enviandos diretamente para o usuário para ajudar na solução do problema.

- **Exemplo:**

```javascript
const handleCreateUser = async (message: Message, client: Client) => {
	try {
		message.reply(`Olá, eu sou o Vallet, tudo bem?`);
		await delay(1000);
		client.sendMessage(message.from, "Para que eu possa te ajudar, preciso que você crie uma conta em nosso sistema.");
		await delay(1000);
		client.sendMessage(message.from, "Para isso, preciso que você me informe seu nome completo, por favor.");
		const newUser: PrismaUser = {
			id: uuidv4(),
			name: "",
			cellPhone: message.from,
			requestState: 0,
			role: [Role.LEAD],
			createdAt: new Date()
		};
		userService.createAccountUser(newUser);
	} catch (error: any) {
		console.error("An error occured", error);
		message.reply("An error occured, please contact the administrator. (" + error.message + ")");
	}
};
```

O código acima é responsavel por criar um novo usuário no sistema, caso ocorra algum erro, o `catch` irá tratar o erro e enviar uma mensagem para o usuário.

### Logs de Interação do Usuário no Chatbot

Os logs de interação do usuário são rastreados por meio de uma variavel presente no banco de dados, chamada `requestState` que é responsavel por armazenar o estado atual do usuário. Essa variavel é atualizada a cada interação do usuário com o chatbot. Esses logs pode ser usuado para a analise de dados, pois é possivel saber quais são as interações mais comuns do usuário com o chatbot.

- **Legenda dos estados do USER:**

1. `0`: Estado inicial do usuário sem cadastro.
2. `1`: Estado de espera de resposta do usuário com o menu.
3. `2`: Estado de espera de confirmação do usuário com o menu.
4. `3`: Estado de espera de confirmação de nova peça.
5. `4`: Estado de espera de confirmação de status de pedido.
6. `5`: Estado de espera de confirmação de cancelar pedido.
7. `6`: Estado de espera de confirmação de atualizar nome do usuário.

- **Legenda dos estados do LEAD:**

1. `0`: Estado inicial do usuário sem cadastro.

- **Legenda dos estados do ADMIN:**

1. `0`: Estado inicial do usuário sem cadastro.
2. `1`: Estado de espera de resposta do usuário com o menu.
3. `2`: Estado de espera de confirmação do usuário com o menu.
4. `3`: Estado de espera de confirmação de nova ponto.
5. `4`: Estado de espera de confirmação de dar acesso a um usuário.

### Logs das Interações dos Nós ROS

Os nós ROS2 são uma parte integral do projeto, principalmente no controle e movimentação do Turtlebot3 com o uso do pacote Nav2. Para o gerenciamento eficaz das ações que cada nó esta fazendo, foi desenvolvido um nó específico para registrar as atividades. O nó [`Logger`](https://github.com/2023M8T2-Inteli/grupo4/blob/feature/logs2/src/bridge/src/vallet/vallet/logger.py) é responsável por coletar e armazenar logs de diferentes nós ROS2. A seguir estão os detalhes de sua implementação e funcionalidade.

#### Implementação do Nó `Logger`

O nó `Logger` foi implementado em Python e faz parte do nosso pacote "vallet" para ROS2. Ele é responsável por se inscrever em um tópico específico e registrar os dados recebidos em um arquivo `.txt`. A estrutura e o funcionamento do nó `Logger` são apresentados abaixo:

- **Inscrição no Tópico de Logs**: O nó `Logger` inscreve-se no tópico `/logs` para receber mensagens do tipo `Log`, definidas no pacote `vallet_msgs`.

- **Callback de Log**: Quando uma mensagem é recebida, o callback `log_callback` é invocado. Este callback processa a mensagem de log, extraindo informações como o nome do nó, a ação realizada e o horário do evento (Unix timestamp).

- **Armazenamento de Logs**: As informações recebidas são formatadas e escritas em um arquivo de log. O arquivo é criado e gerenciado pelo próprio nó, garantindo que todos os logs sejam registrados para análises futuras.

- **Gerenciamento de Arquivos de Log**: O nó `Logger` cria e gerencia um arquivo `temp_data.txt` localizado no diretório `assets` do pacote. Este arquivo é utilizado para armazenar os logs de forma persistente.

#### Planejamento

Hoje, o nó `Logger` é responsável apenas por armazenar localmente os logs recebidos. No entanto, ele pode ser facilmente expandido, e já nos planejamos, para enviar os logs para um servidor remoto, como nosso banco de dados, por exemplo. Essa expansão já está sendo planejada e será implementada na próxima sprint.
