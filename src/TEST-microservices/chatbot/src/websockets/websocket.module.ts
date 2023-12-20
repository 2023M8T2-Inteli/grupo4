import { Module, forwardRef } from '@nestjs/common';
import { WebsocketService } from './websocket.service';
import { WebsocketGateway } from './websocket.gateway';
import { PrismaModule } from 'src/prisma/prisma.module';
import { WhatsappModule } from 'src/whatsapp/whatsapp.module';
import { WebsocketController } from './websockets.controller';

@Module({
  imports: [PrismaModule, forwardRef(()=> WhatsappModule)],
  providers: [WebsocketGateway, WebsocketService],
  exports: [WebsocketGateway, WebsocketService],
  controllers: [WebsocketController],
})
export class WebsocketModule {}
