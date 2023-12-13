import { Module } from '@nestjs/common';
import { HandlerService } from './handler.service';
import { HandlerController } from './handler.controller';
import { AIModule } from '../AI/AI.module';
import { PrismaModule } from '../prisma/prisma.module';

@Module({
  imports: [AIModule, PrismaModule],
  controllers: [HandlerController],
  providers: [HandlerService],
  exports: [HandlerService],
})
export class HandlerModule {}
