import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { PrismaModule } from './prisma/prisma.module';
import { OpenaiModule } from './openai/openai.module';
import { WhatsappModule } from './whatsapp/whatsapp.module';
import { HandlerModule } from './handler/handler.module';

@Module({
  imports: [PrismaModule, OpenaiModule, WhatsappModule, HandlerModule],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
