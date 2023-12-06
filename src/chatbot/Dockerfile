# Escolher a imagem base do Node.js
FROM node:latest

# Definir o diretório de trabalho no container
WORKDIR /usr/src/app

# Copiar o arquivo 'package.json' e, se disponível, 'package-lock.json'
COPY package*.json ./

RUN npm install

RUN npm install vite-node

RUN npm install -g prisma

RUN npm install -g @prisma/client

COPY . .

RUN prisma generate

EXPOSE 3000

CMD ["npm", "run", "start"]
