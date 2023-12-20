FROM node:bullseye-slim

RUN apt update
# components for whatsapp-web.js (support no-gui systems)
RUN apt install -y gconf-service libgbm-dev libasound2 libatk1.0-0 libc6 libcairo2 libcups2 libdbus-1-3 libexpat1 libfontconfig1 libgcc1 libgconf-2-4 libgdk-pixbuf2.0-0 libglib2.0-0 libgtk-3-0 libnspr4 libpango-1.0-0 libpangocairo-1.0-0 libstdc++6 libx11-6 libx11-xcb1 libxcb1 libxcomposite1 libxcursor1 libxdamage1 libxext6 libxfixes3 libxi6 libxrandr2 libxrender1 libxss1 libxtst6 ca-certificates fonts-liberation libappindicator1 libnss3 lsb-release xdg-utils wget
RUN apt install -y chromium

# For transcription
RUN apt install -y ffmpeg

# Definir o diretório de trabalho no container
WORKDIR /usr/src/app

# Copiar o arquivo 'package.json' e, se disponível, 'package-lock.json'
COPY package*.json ./

COPY . .

RUN cd src/ && npm install

RUN cd src/ && npm install vite-node

RUN cd src/ &&  npm install -g prisma

RUN cd src/ && npm install -g @prisma/client

RUN cd src/ && npx prisma generate

EXPOSE 3000

EXPOSE 3030

CMD ["npm", "run", "start"]
