FROM node:bullseye-slim

# Install dependencies for whatsapp-web.js and Puppeteer
RUN apt update && apt install -y \
   gconf-service libgbm-dev libasound2 libatk1.0-0 libc6 libcairo2 libcups2 libdbus-1-3 \
   libexpat1 libfontconfig1 libgcc1 libgconf-2-4 libgdk-pixbuf2.0-0 libglib2.0-0 libgtk-3-0 \
   libnspr4 libpango-1.0-0 libpangocairo-1.0-0 libstdc++6 libx11-6 libx11-xcb1 libxcb1 \
   libxcomposite1 libxcursor1 libxdamage1 libxext6 libxfixes3 libxi6 libxrandr2 libxrender1 \
   libxss1 libxtst6 ca-certificates fonts-liberation libappindicator1 libnss3 lsb-release \
   xdg-utils wget chromium ffmpeg

RUN useradd -m myuser

# Set the working directory in the container
WORKDIR /usr/src/app
RUN chown -R myuser:myuser /usr/src/app

USER myuser

# Copy 'package.json' and 'package-lock.json', if available
COPY package*.json ./

# Install project dependencies
RUN npm ci

USER root
# Copy the rest of the project files
COPY . .
RUN chown -R myuser:myuser /usr/src/app

USER myuser

# Set Puppeteer executable path to the installed Chromium
ENV PUPPETEER_EXECUTABLE_PATH=/usr/bin/chromium

# Generate Prisma client
RUN cd src/ && npx prisma generate

# Expose ports
EXPOSE 3000
EXPOSE 3030

CMD ["npm", "run", "start"]
