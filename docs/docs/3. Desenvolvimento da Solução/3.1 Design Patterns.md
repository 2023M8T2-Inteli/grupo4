# Design Patterns

Design Patterns, ou Padrões de Projeto, são soluções típicas para problemas comuns em design de software. Eles representam melhores práticas usadas por desenvolvedores experientes para resolver problemas similares. A utilização de design patterns ajuda a tornar o código mais modular, flexível e reutilizável, além de facilitar a comunicação entre desenvolvedores ao oferecer uma linguagem comum.

## Exemplos de Utilização de Design Patterns

### Código de Navegação do Turtlebot3


- **Arquivo escolhido:** `vallet.py` 
- **Path:** `src/bridge/src/navigation_turtlebot3/navigation_turtlebot3/vallet.py`

1. **Observer Pattern**: 
   - **Definição**: Este padrão é um estilo de comunicação onde os remetentes de mensagens (conhecidos como subjects) mantém uma lista de dependentes, os observadores. Esses dependentes se atualizam com base em notificações de mudança de estado enviadas pelo subject.
   - **Aplicação no Código**: No código do Turtlebot3, o `Publisher` e `Subscriber` são usados para gerenciar a comunicação entre diferentes partes do sistema de navegação. Por exemplo, o status do robô é publicado pelo `Publisher` (que assume o papel de subject), e diversos `Subscribers` estão configurados para receber e agir com base nas posições.

2. **Command Pattern**:
   - **Definição**: Este padrão encapsula uma solicitação como um objeto, permitindo parametrizar clientes com diferentes solicitações, enfileirar ou registrar solicitações e implementar operações reversíveis. É particularmente útil para desacoplar o objeto que invoca a operação do objeto que sabe como executar a operação.
   - **Aplicação no Código**: A classe `NavigatorController` atua como um invocador de comandos, delegando ações específicas para o objeto `navigator`. Por exemplo, o método `go_to_pose` é uma abstração de um comando que é executado pelo `navigator`.

### Código do Chatbot no WhatsApp

- **Arquivo escolhido:** `message.ts` 
- **Path:** `src/chatbot/src/handlers/message.ts`

1. **Chain of Responsibility Pattern**:
   - **Definição**: Este padrão permite passar a solicitação ao longo de uma cadeia de manipuladores. Ao receber uma solicitação, cada manipulador decide processar a solicitação ou passá-la para o próximo manipulador na cadeia.
   - **Aplicação no Código**: As classes `MessageValidator`, `GroupMessageValidator`, e `BotReadyValidator` exemplificam esse padrão. Cada validator na cadeia tem a oportunidade de processar a mensagem e, dependendo de sua lógica, pode parar a cadeia ou passar para o próximo validator.

2. **Command Pattern**:
   - **Aplicação no Código**: Este padrão é aplicado nas classes `RequestUserHandler` e `RequestAdminHandler`, onde cada uma encapsula a lógica de como responder a diferentes tipos de requisições. O método `handle` em cada uma dessas classes representa um comando específico que é executado de acordo com o estado da requisição.

3. **Factory Method Pattern** (Inferido):
   - **Definição**: Este padrão fornece uma interface para criar objetos em uma superclasse, mas permite que as subclasses alterem o tipo de objetos que serão criados. É usado quando uma classe não pode antecipar a classe de objetos que deve criar.
   - **Aplicação no Código**: Embora não explicitamente definido, o padrão pode ser inferido na maneira como diferentes handlers (como `RequestUserHandler`, `RequestAdminHandler`, `RequestLeadHandler`) são criados e usados com base no tipo de usuário. Cada handler pode ser visto como um "produto" criado por uma "fábrica" abstrata, que no caso é a lógica que decide qual handler instanciar com base no papel do usuário.

### Conclusão

A aplicação de design patterns nos códigos analisados demonstra uma abordagem sofisticada para resolver problemas de design de software. No Turtlebot3, o uso do Publisher-Subscriber e Command facilita a comunicação e a execução de ações complexas de forma desacoplada. No chatbot do WhatsApp, o Chain of Responsibility gerencia o fluxo de mensagens, enquanto o Command centraliza o controle de requisições. A inferência do Factory Method mostra como diferentes componentes são criados dinamicamente. Esses padrões não só promovem a eficiência e a manutenção do código, mas também facilitam a expansão e a adaptabilidade do sistema.
