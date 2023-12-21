# Arquitetura da Solução

## Introdução

Propomos uma solução integrada de automação utilizando o TurtleBot 3, um robô amplamente equipado com ferramentas de navegação, interação auditiva e visualização. O sistema terá como principal objetivo facilitar e agilizar o processo de coleta e entrega de equipamentos do almoxarifado da AMBEV. Esta seção detalha as funcionalidades e o fluxo operacional do sistema proposto.

## Função Principal

A principal função do TurtleBot 3 é realizar a rota otimizada necessária para o transporte de um objeto. Ao ser instruído para coletar um item, ele utiliza algoritmos de otimização para determinar o caminho mais eficiente até o destino. Esta abordagem garante que o robô minimize o tempo de viagem, reduza possíveis obstáculos e maximize a eficácia de cada transporte. Depois de concluir a entrega de um objeto, o TurtleBot 3 retorna à sua posição de espera, pronto para processar o próximo item da fila. Durante cada tarefa, o robô pode receber comandos de voz do usuário para iniciar ou adaptar sua ação, proporcionando uma interação dinâmica e responsiva.

## Integração com ROS e EC2

A orientação de deslocamento para o robô é fornecida através de uma "ROS bridge". Este é um computador local rodando um nó ROS e também um servidor, responsável por se comunicar com uma instância EC2 na nuvem. A instância EC2 organiza os comandos recebidos em uma fila e distribui-os entre os robôs disponíveis, garantindo eficiência na execução das tarefas.

## Processamento de Comandos de Voz

Quando um comando de voz é emitido ao TurtleBot 3, este áudio é enviado para a instância EC2. Utilizando um serviço de Speech-to-Text (STT), o áudio é convertido em texto e processado. Uma vez compreendido o comando, uma resposta é gerada e, por meio de um serviço de Text-to-Speech (TTS), é convertida em áudio e enviada de volta ao robô para informar o usuário ou iniciar a ação correspondente.

## Interface do Cliente

Uma interface de usuário está em desenvolvimento, permitindo que os clientes solicitem uma peça ou item específico. Esta interface poderá ser um aplicativo independente ou até mesmo uma integração com o WhatsApp. Assim que o cliente faz um pedido, o processo de enviar o robô até a localização do item e transportá-lo é iniciado.

## Diagrama de Blocos - Versão 1

<p align="center">
      <img src={require('@site/static/img/diagrama-de-blocos-v1.png').default} alt="Diagrama de blocos" />
</p>

Para visualização completa, [acesse o link aqui.](https://www.figma.com/file/SgEvZR8sd8eom18jGBZDoI/Diagrama-de-blocos?type=whiteboard&node-id=0%3A1&t=TaLMiltzfbKt9zS3-1)
