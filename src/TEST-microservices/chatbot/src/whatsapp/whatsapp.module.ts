import { Module } from '@nestjs/common';
import { WhatsappService } from './whatsapp.service';
import { WhatsappController } from './whatsapp.controller';
import { HandlerModule } from '../handler/handler.module';
import { PrismaModule } from '../prisma/prisma.module';

@Module({
  imports: [HandlerModule, PrismaModule],
  controllers: [WhatsappController],
  providers: [WhatsappService],
  exports: [WhatsappService],
})
export class WhatsappModule {}
