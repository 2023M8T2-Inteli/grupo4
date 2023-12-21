import { Module } from '@nestjs/common';
import { InterfaceController } from './interface.controller';
import { InterfaceService } from './interface.service';
import { PrismaModule } from 'src/prisma/prisma.module';
import { AIService } from 'src/AI/AI.service';

@Module({
  controllers: [InterfaceController],
  providers: [InterfaceService, AIService],
  imports: [PrismaModule],
})
export class InterfaceModule {}
