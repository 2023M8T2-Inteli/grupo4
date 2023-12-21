import { Module } from '@nestjs/common';
import { PrismaService } from './prisma.service';
import { OrderService } from './order.service';
import { UserService } from './user.service';
import { ToolService } from './tool.service';
import { LocationService } from './location.service';
import { TranscriptionService } from './transcription.service';
import { GPTFunctionCallingService } from './GPTFunctionCalling.service';

@Module({
  providers: [
    PrismaService,
    OrderService,
    UserService,
    ToolService,
    LocationService,
    TranscriptionService,
    GPTFunctionCallingService,
  ],
  exports: [
    UserService,
    PrismaService,
    OrderService,
    ToolService,
    LocationService,
    TranscriptionService,
    GPTFunctionCallingService,
  ],
})
export class PrismaModule {}
