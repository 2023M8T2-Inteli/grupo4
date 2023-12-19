import { Module, forwardRef } from '@nestjs/common';
import { HandlerService } from './handler.service';
import { HandlerController } from './handler.controller';
import { AIModule } from '../AI/AI.module';
import { PrismaModule } from '../prisma/prisma.module';
import { HandleUserService } from './handle-user.service';
import { HandleLeadService } from './handle-lead.service';
import { HandleAdminService } from './handle-admin.service';
import { WhatsappModule } from 'src/whatsapp/whatsapp.module';
import { WebsocketModule } from 'src/websockets/websocket.module';

@Module({
  imports: [AIModule, PrismaModule, forwardRef(() => WhatsappModule), WebsocketModule],
  controllers: [HandlerController],
  providers: [
    HandlerService,
    HandleUserService,
    HandleLeadService,
    HandleAdminService,
  ],
  exports: [HandlerService],
})
export class HandlerModule {}
