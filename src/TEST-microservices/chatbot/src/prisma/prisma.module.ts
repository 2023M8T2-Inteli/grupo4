import { Module } from '@nestjs/common';
import { PrismaService } from './prisma.service';
import { OrderService } from './order.service';
import { UserService } from './user.service';
import { PointService } from './coordinates.service';

@Module({
  providers: [PrismaService, OrderService, UserService, PointService],
  exports: [UserService, PrismaService, OrderService, PointService],
})
export class PrismaModule {}
