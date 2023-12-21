# Banco de Dados

Um **banco de dados** é um conjunto de dados organizados de forma estruturada, armazenados eletronicamente em um sistema de computador. Eles são essenciais para armazenar e gerenciar grandes quantidades de informações de forma eficiente.

## O que é um ORM?

**ORM** (Object-Relational Mapping) é uma técnica de programação que converte dados entre sistemas incompatíveis usando objetos orientados a objetos. Facilita a manipulação de dados em bancos de dados relacionais.

## Introdução ao Prisma

**Prisma** é um ORM moderno para Node.js e TypeScript, que simplifica o processo de consulta e manipulação de dados em bancos de dados. Ele proporciona um ambiente de desenvolvimento mais intuitivo e seguro.

## Uso do PostgreSQL com Prisma

Este projeto utiliza **PostgreSQL**, um sistema de banco de dados relacional, com o backend. Prisma oferece suporte completo para PostgreSQL, permitindo operações de banco de dados eficientes e seguras.

## Conectando-se ao Banco de Dados

Para se conectar ao banco de dados, Prisma utiliza uma URL de conexão, definida na chave `DATABASE_URL` no arquivo `.env`.

```javascript
datasource db {
  provider = "postgresql"
  url      = env("DATABASE_URL")
}
```

A URL de conexão é definida no arquivo `.env` como:

```javascript
DATABASE_URL = "postgresql://janedoe:mypassword@localhost:5432/mydb?schema=sample"
```

## Executando Comandos Prisma

Para executar comandos Prisma, é necessário instalar o Prisma CLI, que pode ser instalado e no terminal estar na pasta `chatbot/src`.

### Prisma Generate

O comando `prisma generate` gera ou atualiza o cliente Prisma, que é usado para acessar o banco de dados em seu código de aplicativo.

```
npx prisma generate
```

### Prisma DB Push

O comando `prisma db push` é usado para atualizar o esquema do banco de dados para corresponder ao esquema Prisma definido em seu arquivo de esquema Prisma.

```
npx prisma db push
```

## Diagrama do Banco de Dados - v1.0

![Alt text](../../static/img/prisma-erd-v1.svg)

## Estrutura do Modelo de Dados

A arquitetura do esquema de banco de dados neste projeto é meticulosamente projetada para otimizar a gestão e o acesso aos dados. Ela incorpora modelos chave, cada um com atributos específicos e funções distintas:

**User:** Este modelo é fundamental para armazenar dados pessoais e profissionais dos usuários. Inclui campos como ID (identificador único), Nome (nome do usuário), Celular (contato telefônico), Estado da Solicitação (status atual do usuário no sistema) e Papel (definindo a função do usuário, como administrador ou usuário regular).

**Tool:** Representa as ferramentas ou recursos disponíveis. Cada entrada neste modelo contém ID (identificação única da ferramenta), Código (código identificador específico da ferramenta), Nome, Preço, Tags (etiquetas para categorização) e informações sobre a disponibilidade, como Quantidade Mínima e Quantidade Máxima em estoque.

**Order:** Este modelo é essencial para rastrear pedidos. Ele associa usuários às ferramentas, contendo detalhes de cada pedido realizado. A estrutura inclui relações com os modelos User e Tool, facilitando a visualização e o gerenciamento das ordens.

**Point:** Define locais ou pontos de interesse. Cada ponto é caracterizado por um ID único, Nome e Coordenadas geográficas, permitindo uma localização precisa e gestão eficaz de locais relacionados ao projeto.

**Role:** Uma enumeração que categoriza os diferentes papéis atribuídos aos usuários dentro do sistema. Inclui designações como LEAD, USER e ADMIN, cada uma com permissões e responsabilidades específicas, vital para a segurança e eficiência na gestão de acessos e funções dos usuários.

- **Observação:** O campo `requestState` do modelo User é uma enumeração que define o estado atual do usuário no sistema. Esse campo não é uma tabela separada, devido á arquitetura de construção do Chatbot, que funciona a partir de uma fila de espera e funções assincronas. Ao qual não permite a criação de uma tabela para armazenar os estados dos usuários, pois o poderia gerar um travamento de processamento do sistema.
- 
## Conclusão

O banco de dados é um componente essencial para o projeto, atuando como o epicentro para armazenar e gerenciar dados de forma eficiente e segura. A escolha do PostgreSQL, em particular, destaca-se pela sua robustez, desempenho e confiabilidade, características fundamentais para o suporte de operações complexas e de grande volume de dados. O Prisma, como uma ferramenta de ORM, eleva essa eficiência, simplificando o processo de consulta e manipulação de dados em bancos de dados relacionais. Ele oferece um ambiente de desenvolvimento mais intuitivo, seguro e adaptável, alinhando-se perfeitamente com as funcionalidades avançadas do PostgreSQL. Juntos, eles formam uma combinação poderosa, garantindo um gerenciamento de dados otimizado e uma base sólida para o desenvolvimento sustentável do projeto.
