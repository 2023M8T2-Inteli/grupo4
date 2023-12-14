import { Module } from '@nestjs/common';
import { HandlerService } from './handler.service';
import { HandlerController } from './handler.controller';
import { AIModule } from '../AI/AI.module';
import { PrismaModule } from '../prisma/prisma.module';
import { HandleUserService } from './handle-user.service';
import { HandleLeadService } from './handle-lead.service';
import { HandleAdminService } from './handle-admin.service';

@Module({
  imports: [AIModule, PrismaModule],
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
