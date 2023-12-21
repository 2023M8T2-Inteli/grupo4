import { Module } from '@nestjs/common';
import { PrismaModule } from './prisma/prisma.module';
import { AIModule } from './AI/AI.module';
import { WhatsappModule } from './whatsapp/whatsapp.module';
import { HandlerModule } from './handler/handler.module';
import { ConfigModule } from '@nestjs/config';
import { WebsocketModule } from './websockets/websocket.module';
import { InterfaceModule } from './interface/interface.module';

@Module({
  imports: [
    PrismaModule,
    AIModule,
    HandlerModule,
    WhatsappModule,

    ConfigModule.forRoot({
      isGlobal: true,
    }),

    WebsocketModule,

    InterfaceModule,
  ],
})
export class AppModule {}
