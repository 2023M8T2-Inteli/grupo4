# Estruturas de Dados

Na nossa solução para navegação autônoma do TurtleBot utilizando o Navigation Stack 2 (Nav2) do ROS, fazemos uso de duas estruturas de dados principais para coordenar e gerenciar o movimento do robô: uma fila (queue) e uma árvore de decisões.

## Implementação da Fila

A fila é implementada na classe `PoseQueue`, localizada no arquivo `pose_queue.py`. Essa fila é responsável por armazenar dados de poses (coordenadas x, y) que o robô precisa navegar. A fila garante uma execução sistemática e organizada das tarefas de navegação. A classe `PoseQueue` é definida da seguinte forma:

```python

class PoseQueue(Node):
    def __init__(self):
        # ... (outro código de inicialização)
        self.queue = Queue()
        self.robot_status = RobotStatus.FREE
        self.emergency_stop_state = EmergencyStopState.DISABLED

    def enqueue_controller(self, msg: dict) -> None:
        # ... (outro código)
        if 'stop' in json_msg:
            self._handle_stop_data(json_msg)
        elif 'x' in json_msg and 'y' in json_msg:
            self._handle_pose_data(json_msg)
        else:
            raise TypeError(f"Tipo inválido: {type(json_msg)}, mensagem: {json_msg}")

    def _handle_pose_data(self, json_msg: dict) -> None:
        # ... (outro código)
        if x is not None and y is not None and self.emergency_stop_state == EmergencyStopState.DISABLED:
            # ... (outro código)
            self.queue.put(pose)
        elif self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("Parada de emergência é acionável")
        else:
            self.get_logger().info(f"Dados de pose inválidos: {json_msg}")

    def status_callback(self, msg: String) -> None:
        # ... (outro código)
        if self.robot_status == RobotStatus.FREE and self.emergency_stop_state == EmergencyStopState.DISABLED and not self.queue.empty():
            self.dequeue.publish(self.queue.get())
        elif self.robot_status == RobotStatus.BUSY and self.emergency_stop_state == EmergencyStopState.DISABLED:
            self.get_logger().info("O robô está ocupado")
        elif self.robot_status == RobotStatus.BUSY and self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("O robô está ocupado e a parada de emergência é acionável")    
        elif self.queue.empty() and self.robot_status == RobotStatus.FREE and self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("O robô está livre e a parada de emergência é acionável e a fila está vazia")

```

- O construtor `__init__` inicializa diversos elementos essenciais, como os tópicos de comunicação, o estado do robô (`robot_status`), o estado da parada de emergência (`emergency_stop_state`), e a própria fila (`queue`).

- O método `enqueue_controller` é chamado quando novas mensagens são recebidas no tópico de enfileiramento (`/enqueue`). Ele analisa a mensagem JSON recebida para determinar se é uma solicitação de parada de emergência ou uma posição de destino para o robô.

- Os métodos `_handle_stop_data` e `_handle_pose_data` tratam respectivamente das solicitações de parada de emergência e das posições de destino. Em caso de posição de destino, a pose é convertida para o formato adequado e enfileirada, além de ser registrada em um log.

- O método `status_callback` é chamado quando há uma atualização do status do robô. Ele verifica as condições para desenfileirar uma nova pose, levando em consideração o estado do robô e da parada de emergência.

## Integração da Árvore de Decisões

A árvore de decisões é uma parte crucial da navegação autônoma, mas no contexto do Nav2, sua implementação é incorporada ao framework e não explicitamente codificada no nosso script. Em termos gerais:

- O Nav2 utiliza plugins de planejamento e controle, que incluem uma árvore de decisões para guiar o robô em ambientes desconhecidos, evitando obstáculos e planejando trajetórias seguras.

- A referência à árvore de decisões no Nav2 é feita através de configurações e parâmetros específicos que são definidos nos arquivos de configuração do Nav2, como `nav2_params.yaml`.

Essas estruturas de dados trabalham em conjunto para proporcionar um sistema organizado e seguro de navegação autônoma para o TurtleBot, utilizando uma fila para gerenciar destinos específicos e a árvore de decisões do Nav2 para abordar aspectos mais complexos da navegação, como desvios de obstáculos e planejamento de trajetória.

