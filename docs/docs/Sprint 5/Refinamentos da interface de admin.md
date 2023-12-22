# Refinamentos na UI de Admin

Durante a quinta sprint, concentramos nossos esforços em adicionar as últimas funcionalidades, algumas já comentadas em sprints anteriores, mas não implementadas, para enriquecer a interface de administração.

## Integração do QR Code

A seção de QR Code, disponível na sidebar, visa facilitar a autenticação do administrador ao acessar a aplicação. A integração ocorre por meio de um endpoint na AWS, que fornece um objeto com duas chaves: 

- `isAuthenticated`: Um valor binário indicando se o usuário está autenticado.
- `qr`: Contém um SVG do QR Code. Se o usuário já estiver autenticado, essa chave estará vazia, dispensando nova autenticação. Se não estiver autenticado, um QR Code será exibido. Se ambas as chaves `qr` e `isAuthenticated` forem negativas, o usuário é informado da geração pendente do QR Code.

## Download de Dados

É possível baixar dados das tabelas utilizando um botão localizado no canto superior direito de cada tabela. A funcionalidade é implementada pelo componente `DownloadButton`, que requer:

- Os dados da tabela a serem transformados em arquivo.
- O nome da tabela.

Os dados são convertidos em um arquivo CSV (comma-separated values) e disponibilizados para download, podendo ser usados no Excel (como um arquivo XLSX) ou em códigos e arquivos de notebook para análises de dados.

## CRUD das Tabelas

Melhoramos a interface, adicionando funcionalidades de CREATE, UPDATE e DELETE nas tabelas de Usuários, Itens e Destinos.

### Create

- Novo botão "Adicionar Novo" em todas as páginas.
- Um modal permite inserir dados do novo item.
- Ao salvar, a requisição de criação é enviada ao banco e a tabela é atualizada.
- Exceção: A tabela de Usuários não possui essa funcionalidade devido à necessidade de autenticação por número de celular via Whatsapp.

### Update

- Para atualizar, basta dar dois cliques na célula desejada.
- O campo se torna um input editável.
- Para salvar, clique fora da célula.

### Delete

- Para deletar uma linha, clique no ícone de lixeira ao final da linha.
- Isso aciona uma requisição DELETE para o servidor na rota associada.
