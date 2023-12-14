import { Module } from '@nestjs/common';
import { PrismaService } from './prisma.service';
import { OrderService } from './order.service';
import { UserService } from './user.service';
import { ToolService } from './tool.service';
import { LocationService } from './location.service';

@Module({
  providers: [PrismaService, OrderService, UserService, ToolService, LocationService ],
  exports: [UserService, PrismaService, OrderService, ToolService, LocationService],
})
export class PrismaModule {}
