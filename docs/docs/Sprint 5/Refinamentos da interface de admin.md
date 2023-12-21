# Refinamentos na UI de Admin

Durante a quinta sprint, concentramos nossos esforços em adicionar as últimas funcionalidades para enriquecer a interface de administração.

## Integração do QR Code

## Download de dados

## CRUD das tabelas

Aprimorando a interface anterior, adicionamos as funções de CREATE, UPDATE e DELETE nas páginas relacionadas às tabelas de Usuários, Itens e Destinos. 

### Create

Todas as páginas agora possuem um botão de "Adicionar Novo". Ao clicar nele, um modal aparece, no qual é possível preencher os dados do novo item. Ao clicar em salvar, a requisição de criação é enviada ao banco e a tabela no frontend é atualizada.

A única tabela que não possui essa funcionalidade é a de Usuários, pois usuários só podem ser adicionados pelo Whatsapp, pelo requisito de autenticação por número de celular.

### Update

Para atualizar os dados, basta clicar duas vezes na célula que se deseja modificar. Ao fazer isso, o campo se tornará um input editável. Para salvar, basta clicar fora da célula.

### Delete

Para deletar uma linha da tabela, clique no ícone de lixeira na extrema direita da linha. Esse ícone acionará uma função de requisição DELETE para o servidor, na rota associada.

