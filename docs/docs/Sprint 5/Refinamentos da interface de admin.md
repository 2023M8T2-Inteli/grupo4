# Refinamentos na UI de Admin

Durante a quinta sprint, concentramos nossos esforços em adicionar as últimas funcionalidades, algumas já comentadas em sprint anteriores porém não implementadas, para enriquecer a interface de administração.

## Integração do QR Code

O QR Code é uma das seções disponíveis na sidebar, que possui o objetivo de facilitar a autenticação do administrador ao acessar a aplicação. A integração com QR Code é feita acessando um endpoint gerado no Back-end, que está na AWS. Desse endpoint, ele recebe um objeto com duas chaves: 'isAuthenticated', um binário para saber se o usuário já está autenticado e 'qr', que contém um SVG do QR Code. Caso o usuário já esteja autenticado no sistema, ele recebe a chave 'qr' vazia, pois dispensa uma nova autenticação. Se ele não estiver, um QR Code é exibido na tela. Caso a chave 'qr' esteja vazia e 'isAuthenticated' falso, o usuário será informado que ainda será gerado o QR Code.

## Download de dados

O download de dados planilhas é possível por meio de um botão que se situa no canto superior direito de cada tabela. Ele foi feito por meio de um componente chamado 'DownloadButton', que recebe como argumento os dados da tabela que será transformada em um arquivo e o nome dela. Os dados são todos transformados em arquivo csv(comma-separeted-values) e baixados na máquina da pessoa, pondendo utilizar esse arquivo tanto no Excel, transformando em um arquivo xlsx(Microsoft Excel Spreadsheet), ou utilizá-lo em códigos ou arquivos notebook, para fazer análise de dados e obter ainda mais insights da operação.

## CRUD das tabelas

Aprimorando a interface anterior, adicionamos as funções de CREATE, UPDATE e DELETE nas páginas relacionadas às tabelas de Usuários, Itens e Destinos.

### Create

Todas as páginas agora possuem um botão de "Adicionar Novo". Ao clicar nele, um modal aparece, no qual é possível preencher os dados do novo item. Ao clicar em salvar, a requisição de criação é enviada ao banco e a tabela no frontend é atualizada.

A única tabela que não possui essa funcionalidade é a de Usuários, pois usuários só podem ser adicionados pelo Whatsapp, pelo requisito de autenticação por número de celular.

### Update

Para atualizar os dados, basta clicar duas vezes na célula que se deseja modificar. Ao fazer isso, o campo se tornará um input editável. Para salvar, basta clicar fora da célula.

### Delete

Para deletar uma linha da tabela, clique no ícone de lixeira na extrema direita da linha. Esse ícone acionará uma função de requisição DELETE para o servidor, na rota associada.
