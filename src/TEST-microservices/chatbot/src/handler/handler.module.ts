import { Module } from '@nestjs/common';
import { HandlerService } from './handler.service';
import { HandlerController } from './handler.controller';

@Module({
  controllers: [HandlerController],
  providers: [HandlerService],
})
export class HandlerModule {}
