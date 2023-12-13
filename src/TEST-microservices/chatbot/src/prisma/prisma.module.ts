import { Module } from '@nestjs/common';
import { PrismaService } from './prisma.service';
import { OrderService } from './order.service';
import { UserService } from './user.service';

@Module({
  providers: [PrismaService, OrderService, UserService],
  exports: [UserService, PrismaService, OrderService],
})
export class PrismaModule {}
