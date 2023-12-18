import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { PrismaModule } from './prisma/prisma.module';
import { AIModule } from './AI/AI.module';
import { WhatsappModule } from './whatsapp/whatsapp.module';
import { HandlerModule } from './handler/handler.module';
import { ConfigModule } from '@nestjs/config';

@Module({
  imports: [
    PrismaModule,
    AIModule,
    HandlerModule,
    WhatsappModule,

    ConfigModule.forRoot({
      isGlobal: true,
    }),
  ],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
