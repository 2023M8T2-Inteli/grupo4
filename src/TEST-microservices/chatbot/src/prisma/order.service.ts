import { Inject, Injectable } from '@nestjs/common';
import { Order as PrismaOrder } from '@prisma/client';
import { PrismaService } from './prisma.service';
import { UserService } from './user.service';

class NotPossibleToCreateOrder extends Error {
  constructor(message: string = 'Not possible to create order') {
    super(message);
    this.name = 'NotPossibleToCreateOrder';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, NotPossibleToCreateOrder);
    }
  }
}

@Injectable()
export class OrderService {
  constructor(
    @Inject(PrismaService) private prisma: PrismaService,
    @Inject(UserService) private userService: UserService,
  ) {}

  async getOrderByCode(
    userPhone: string,
    order_: string | number,
  ): Promise<PrismaOrder | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: userPhone },
      });
      if (user != null) {
        const order = await this.prisma.order.findFirst({
          where: {
            code: Number(order_),
            userId: user.id,
          },
        });
        if (order == null) {
          return null;
        }
        return order;
      } else {
        return null;
      }
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    }
  }

  async createOrder(
    cellPhone: string,
    toolId: string,
    pointId: string,
  ): Promise<PrismaOrder> {
    const user = await this.userService.getUser(cellPhone);
    const orders = await this.prisma.order.create({
      data: {
        type: 'In Progress',
        toolId: toolId,
        userId: user.id,
        pointId: pointId,
      },
    });
    if (orders == null) {
      throw new NotPossibleToCreateOrder();
    }
    return orders;
  }

  async updateOrder(
    cellPhone: string,
    pointId: string,
  ): Promise<number | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: cellPhone },
      });
      if (user != null) {
        const order = await this.prisma.order.updateMany({
          where: {
            userId: user.id,
            type: 'In Progress',
            pointId: '123e4567-e89b-12d3-a456-426614174013',
          },
          data: { pointId: pointId },
        });
        if (order == null) {
          return null;
        }
        return order.count;
      } else {
        return null;
      }
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    }
  }

  async verifyOpenOrder(userPhone: string): Promise<PrismaOrder | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: userPhone },
      });
      if (user != null) {
        const orders = await this.prisma.order.findFirst({
          where: {
            pointId: '123e4567-e89b-12d3-a456-426614174013',
            userId: user.id,
          },
        });
        if (orders == null) {
          return null;
        }
        return orders;
      }
      return null;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    }
  }
  async getOpenOrder(userPhone: string): Promise<PrismaOrder[] | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: userPhone },
      });
      if (user != null) {
        const orders = await this.prisma.order.findMany({
          where: {
            type: 'In Progress',
            userId: user.id,
          },
        });
        if (orders == null) {
          return null;
        }
        return orders;
      }
      return null;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    }
  }

  async cancelOrder(
    userPhone: string,
    order_: string | number,
  ): Promise<PrismaOrder | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: userPhone },
      });
      if (user) {
        const order = await this.prisma.order.update({
          where: {
            code: Number(order_),
            userId: user.id,
          },
          data: {
            type: 'Canceled',
          },
        });
        if (order == null) {
          return null;
        }
        return order;
      } else {
        return null;
      }
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    }
  }
}
