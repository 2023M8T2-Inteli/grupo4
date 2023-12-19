import { Module } from '@nestjs/common';
import { PrismaService } from './prisma.service';
import { OrderService } from './order.service';
import { UserService } from './user.service';
import { ToolService } from './tool.service';
import { LocationService } from './location.service';
import { TranscriptionService } from './transcription.service';

@Module({
  providers: [
    PrismaService,
    OrderService,
    UserService,
    ToolService,
    LocationService,
    TranscriptionService,
  ],
  exports: [
    UserService,
    PrismaService,
    OrderService,
    ToolService,
    LocationService,
    TranscriptionService,
  ],
})
export class PrismaModule {}
