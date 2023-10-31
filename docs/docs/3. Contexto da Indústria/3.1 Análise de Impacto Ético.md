# Análise de impacto ético

O avanço tecnológico representado pelo robô assistente desenvolvido neste módulo promete uma revolução na maneira como lidamos com problemas técnicos e logísticos em ambientes industriais e comerciais. Ao empregar a Modelagem de Linguagem de Máquina (LLM), o robô não apenas entende o dilema apresentado pelo usuário, mas também oferece soluções prováveis e busca a peça para reposição ou ferramenta no almoxarifado e leva para o técnico. Isso será realizado por meio de uma cesta que deverá ser preenchida pelo responsável do almoxarifado. No entanto, esta inovação não vem sem desafios e implicações, particularmente nas esferas de privacidade de dados e equidade. A interação inicial entre o usuário e o robô, por exemplo, pode exigir a divulgação de informações que, se mal administradas, poderiam comprometer a privacidade do indivíduo. Além disso, o armazenamento seguro e a gestão desses dados é uma preocupação primordial para evitar violações de privacidade.

No que diz respeito à equidade e justiça, a implementação desta tecnologia pode revelar disparidades no acesso e na capacidade de utilizar o robô assistente de maneira eficaz. Por exemplo, a linguagem utilizada pelo robô e o nível de literacia digital necessário para interagir com ele podem ser barreiras para alguns usuários, especialmente aqueles de comunidades marginalizadas ou com menos acesso à educação tecnológica. Além disso, há o risco de que as soluções fornecidas pelo robô possam ser tendenciosas, baseadas nos dados com os quais foi treinado. Por exemplo, se o robô foi treinado principalmente com dados de equipamentos mais modernos e comuns, ele pode ser menos eficaz ao ajudar com problemas em máquinas mais antigas ou menos comuns.

Diante das questões que podem limitar o usuário na capaciade de utilizar o robô, fizemos uma análise com base na pesquisa e abstração do nível de competência digital de cidadãos realizado pela União Europeia. Assim, concluimos que pessoas entre os níves 1 a 3, podem sofrer com a adaptação para utilizar a máquina ou precisarão de instruções detalhadas. Como esses grupos englobam pessoas sem afinidade tecnologica ou até mesmo com deficiência fisica ou conginitiva, pudemos nos organizar melhor para construir uma aplicação mais inclusiva. Segue imagem para referência:


<p align="center">
      <img src={require('@site/static/img/learninglevel.jpg').default} alt="Digital Ocean" />
</p>


Para garantir que o robô assistente seja acessível e inclusivo, várias adaptações são essenciais e estão sendo levadas em considerção para atender às necessidades de usuários com deficiências permanentes ou temporárias. O robô deve ser equipado com tecnologia de voz avançada e reconhecimento de voz para auxiliar usuários com deficiência visual, fornecendo orientações auditivas claras e precisas. Para aqueles com deficiência auditiva, uma interface visual rica com instruções em texto ou sinais visuais, além de compatibilidade com tecnologias de assistência, como aparelhos auditivos, é crucial. Em casos de deficiência motora, o robô deve ser capaz de buscar e entregar itens, além de possuir uma interface de usuário intuitiva e fácil de usar. Para usuários com deficiências cognitivas, o robô deve oferecer instruções simplificadas, passo a passo, e suporte visual, como imagens e vídeos, para facilitar a compreensão. Além disso, a versatilidade do robô é fundamental para adaptar suas funcionalidades e oferecer suporte adequado a usuários que enfrentam incapacidades temporárias, garantindo que todos possam interagir e beneficiar-se do robô de maneira eficaz e respeitosa.

A integração do robô assistente nos processos do almoxarifado foi projetada para ser intuitiva e fluida. Neste módulo pretendemos que o robô sejas capaz de identificar, com precisão, qual as ferramentas solicitadas pelo técnico e comunicar ao almoxarifado, que já pode se preparar para a entrega. Além disso, seu sistema de navegação autônoma permite que ele se desloque ao almoxarifado para carga das peças solicitadas. A economia de tempo e o aumento da eficiência são evidentes, com a redução do tempo que os funcionários gastam procurando e buscando itens. Além disso, o robô pode registrar as transações de inventário em tempo real, mantendo a gestão informada sobre o status atual dos recursos e as necessidades futuras de reposição.

No entanto, a implementação do robô no ambiente do almoxarifado também revela algumas áreas desafiadoras. Por exemplo, imaginamos situações em que o robô encontre dificuldade em interpretar as consultas dos usuários devido à ambiguidade ou terminologia específica utilizada. Isso ressalta a importância da programação de uma linguagem de máquina robusta que possa compreender e responder a uma ampla variedade de consultas. A equipe de desenvolvimento tem trabalhado de forma contínua para refinar e expandir o banco de dados de terminologia e respostas do robô para melhor atender às necessidades dos usuários.

A transparência e o consentimento informado são cruciais para garantir a confiança dos usuários na tecnologia. Isso implica em uma comunicação clara sobre como o robô opera, como os dados do usuário são utilizados, e quais são os benefícios e riscos associados à interação com o robô. A capacidade de o robô explicar suas recomendações e ações de uma maneira compreensível para o usuário não só promove a transparência, mas também pode facilitar a aceitação da tecnologia. O consentimento informado, por outro lado, deve ser uma prática padrão antes de qualquer interação, garantindo que os usuários estejam cientes e confortáveis com a extensão da coleta de dados e de que seus dados serão armazenados de maneira segura.

Outra área de foco é a segurança dos dados. Dado que o robô assistente registra e armazena informações sobre transações de inventário e interações com os usuários, medidas robustas de segurança de dados foram implementadas para garantir a proteção desses dados. Essas medidas incluem criptografia avançada e autenticação de dois fatores para acessar as informações registradas pelo robô. Imaginamos que este projeto servirá como um estudo de caso valioso na interseção da inovação tecnológica, eficiência operacional e responsabilidade ética. A jornada de desenvolvimento e implementação contínua do robô assistente do almoxarifado oferece insights valiosos sobre como as tecnologias emergentes podem ser moldadas e refinadas para melhor servir às necessidades humanas e operacionais, enquanto se mantém em conformidade com as normas éticas e de privacidade.

A responsabilidade social deste projeto é vasta. Ao minimizar o tempo de inatividade de equipamentos e facilitar a localização rápida de peças de reposição, o robô tem o potencial de aumentar significativamente a eficiência operacional e, consequentemente, reduzir o desperdício e a emissão de carbono associada. No entanto, é igualmente importante considerar o impacto ambiental da produção, operação e descarte eventual de todas as partes do robô, como suas rodas, bateria e utensílios. A análise do ciclo de vida do robô, incluindo a origem dos materiais utilizados em sua construção e a eficiência energética durante a operação, é vital para compreender e mitigar o impacto ambiental associado.

Indo um pouco mais afundo sobre o impacto ambiental do robô, a cadeia de produção de suas peças tem sua emissão de carbono e consume energia de diferentes fontes, que podem ou não ser poluentes e devem ser levadas em consideração. Logo devemos analisar para consolidar o gasto energético associado ao robô, que é uma combinação do consumo de energia dos componentes individuais e do sistema como um todo durante a operação. Além disso, a energia necessária para fabricar como citada anteriormente, reciclar ou descartar esses componentes também contribui para o cálculo do gasto energético total.

O descarte e a reciclagem responsáveis dos componentes do robô são essenciais para minimizar o impacto ambiental e garantir a conformidade com as regulamentações locais. Segue uma analise de cada componente da solução:

Em relação ao TurtleBotBurguer3, como é um robô em si, contém várias partes eletrônicas que podem ser recicladas ou descartadas de maneira responsável, principalmente para sua bateria de lítio que deve ser descartada apropriadamente devido a sua decomposição extremamente tóxica e demorada. Embora não tenham diretrizes específicas de reciclagem para o TurtleBot3 Burger, práticas gerais de descarte e reciclagem de eletrônicos podem ser aplicadas【7†(CISA)】.

Sobre os periféricos e ademais como câmera USB 5MP, Display Touch 7pol., Microfone e alto-falante, e Componentes eletrônicos (Jumpers, switches)
Esses componentes são feitos de recursos valiosos como metais, plásticos e vidro, cuja fabricação requer energia significativa【8†(US EPA)】. A reciclagem desses componentes pode incluir a desmontagem, separação mecânica e recuperação de materiais valiosos como ouro, cobre, vidro e alumínio, que podem ser reutilizados, reduzindo assim a necessidade de matéria-prima nova e o descarte de eletrônicos usados【9†(US EPA)】.

Existem programas de reciclagem gratuitos disponíveis onde você pode trazer até sete itens por dia para reciclagem, e alguns programas oferecem recompensas mensais para membros que reciclam regularmente seus eletrônicos【10†(PCMag)】. Além disso, políticas devem ser instituídas para a reforma, reciclagem e descarte responsável de componentes eletrônicos no final de sua vida útil. É crucial seguir as práticas recomendadas de descarte e reciclagem para garantir a gestão responsável dos recursos e minimizar o impacto ambiental.

Em suma, o viés algorítmico e a discriminação são questões complexas que exigem uma atenção considerável. As soluções sugeridas pelo robô podem refletir as limitações e preconceitos presentes nos dados utilizados para treinar o modelo de LLM. Se o conjunto de dados de treinamento incluir principalmente exemplos de determinados tipos de máquinas ou problemas, o robô pode ser menos eficaz ao lidar com situações fora desse escopo. Portanto, é crucial que o desenvolvimento e a implementação desta tecnologia sejam feitos de forma inclusiva e com uma consideração cuidadosa das várias dimensões éticas e socioambientais trabalhadas acima.

**Referências:**

【7†(CISA)】: 

- **Descrição:** A CISA (Cybersecurity & Infrastructure Security Agency) fornece diretrizes sobre como descartar dispositivos eletrônicos de maneira segura e eficaz. Eles destacam a importância de proteger as informações sensíveis contidas nos dispositivos eletrônicos e oferecem métodos para a eliminação segura desses dispositivos. Alguns métodos incluem a exclusão segura de dados, sobrescrição e destruição física do dispositivo.

- **Partes que são recicladas nesse processo:** A CISA não especifica partes particulares de dispositivos eletrônicos que devem ser recicladas. Em vez disso, foca em garantir que os dados sejam removidos de forma segura antes do descarte ou reciclagem. Eles mencionam que dispositivos como computadores, smartphones, tablets e mídias digitais devem ser limpos de informações sensíveis antes de serem descartados ou doados.

- **Link para a referência que comprova essa prática:** [CISA - Proper Disposal of Electronic Devices](https://www.cisa.gov/news-events/news/proper-disposal-electronic-devices)

【8†(EPA)】 e 【9†(EPA)】:
- **Descrição:** A Agência de Proteção Ambiental dos Estados Unidos (EPA) destaca a importância da doação e reciclagem de eletrônicos para ajudar a conservar recursos e materiais naturais. Eletrônicos são feitos de recursos valiosos, incluindo metais, plásticos e vidro, que requerem energia para minerar e fabricar. A doação ou reciclagem de eletrônicos ajuda a conservar esses materiais, evitando a poluição do ar e da água e as emissões de gases de efeito estufa causadas pela fabricação de materiais novos.

- **Partes que são recicladas nesse processo:** 
  - **Eletrônicos em geral:** A reciclagem de um milhão de laptops economiza energia equivalente à eletricidade usada por mais de 3.500 casas nos EUA em um ano.
  - **Celulares:** Para cada milhão de celulares reciclados, 35 mil libras de cobre, 772 libras de prata, 75 libras de ouro e 33 libras de paládio podem ser recuperados.

- **Link para a referência que comprova essa prática:** [EPA - Electronics Donation and Recycling](https://www.epa.gov/recycle/electronics-donation-and-recycling)

【10†(PCMag)】:

- **Descrição:** O artigo da PCMag discute várias maneiras de reciclar ou doar impressoras antigas, que também podem ser aplicadas a outros dispositivos eletrônicos, como robôs e seus componentes. Existem programas e organizações, como Goodwill e The Salvation Army, que aceitam e reciclam equipamentos eletrônicos antigos. Além disso, grandes lojas de eletrônicos, como Staples e Best Buy, oferecem programas de reciclagem onde os dispositivos eletrônicos podem ser descartados de maneira responsável. Algumas dessas lojas também oferecem incentivos, como descontos na compra de novos equipamentos ao reciclar os antigos.

- **Partes que são recicladas nesse processo:** O artigo não especifica partes particulares que são recicladas, mas menciona que os dispositivos são geralmente refurbishados e recolocados em serviço, ou seus materiais são direcionados para os fluxos de reciclagem apropriados. Isso pode incluir a separação e reciclagem de metais, plásticos e outros materiais contidos nos dispositivos.

- **Link para a referência que comprova essa prática:** [PCMag - How to Recycle or Donate Your Old Printer](https://www.pcmag.com/how-to/how-to-recycle-or-donate-your-old-printer)




