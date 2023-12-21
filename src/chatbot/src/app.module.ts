import { Module } from '@nestjs/common';
import { PrismaModule } from './prisma/prisma.module';
import { AIModule } from './AI/AI.module';
import { WhatsappModule } from './whatsapp/whatsapp.module';
import { HandlerModule } from './handler/handler.module';
import { ConfigModule } from '@nestjs/config';
import { WebsocketModule } from './websockets/websocket.module';
import { ServeStaticModule } from '@nestjs/serve-static';
import { join } from 'path';
import { InterfaceModule } from './interface/interface.module';
import { appController } from './app.controller';

@Module({
  imports: [
    PrismaModule,
    AIModule,
    HandlerModule,
    WhatsappModule,

    ConfigModule.forRoot({
      isGlobal: true,
    }),


    ServeStaticModule.forRoot({
      rootPath: join(__dirname, '..', 'src/public'),
    }),

    WebsocketModule,

    InterfaceModule,
  ],
  controllers: [appController]
})
export class AppModule {}


// Made By Grupo 4 - Inteli Modulo 8