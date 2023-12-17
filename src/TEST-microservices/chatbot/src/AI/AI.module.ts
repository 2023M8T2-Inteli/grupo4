import { Module } from '@nestjs/common';
import { AIService } from './AI.service';

@Module({
  providers: [AIService],
  exports: [AIService],
})
export class AIModule {}
