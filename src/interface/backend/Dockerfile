FROM node:bullseye-slim

RUN apt update && apt upgrade -y

# For transcription
RUN apt install -y ffmpeg

# Definir o diretório de trabalho no container
WORKDIR /usr/src/app

# Copiar o arquivo 'package.json' e, se disponível, 'package-lock.json'
COPY package*.json ./

COPY . .

RUN npm install

RUN npm install -g prisma && npm install -g @prisma/client

RUN npx prisma generate

EXPOSE 5000

CMD ["npm", "run", "start"]
