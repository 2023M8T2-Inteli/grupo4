import { Inject, Injectable } from '@nestjs/common';
import { Order as PrismaOrder, Tool as PrismaTool } from '@prisma/client';
import { PrismaService } from './prisma.service';
import { UserService } from './user.service';

export class NotPossibleToCreateOrder extends Error {
  constructor(message: string = 'Not possible to create order') {
    super(message);
    this.name = 'NotPossibleToCreateOrder';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, NotPossibleToCreateOrder);
    }
  }
}

export class OrderDoesntExists extends Error {
  constructor(message: string = 'Order doesnt exists') {
    super(message);
    this.name = 'OrderDoesntExists';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, OrderDoesntExists);
    }
  }
}

export class OrdersEmpty extends Error {
  constructor(message: string = 'Orders empty') {
    super(message);
    this.name = 'OrdersEmpty';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, OrdersEmpty);
    }
  }
}

export class OpenOrdersDoestExist extends Error {
  constructor(message: string = 'Orders empty') {
    super(message);
    this.name = 'OrdersEmpty';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, OrdersEmpty);
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
    const user = await this.userService.getUser(userPhone);

    const order = await this.prisma.order.findFirst({
      where: {
        code: Number(order_),
        userId: user.id,
      },
    });
    if (!order) {
      throw new OrderDoesntExists();
    }
    return order;
  }

  async getAllOrders(userPhone: string) {
    const user = await this.userService.getUser(userPhone);
    const orders = await this.prisma.order.findMany({
      where: {
        userId: user.id,
      },
      include: {
        tool: true,
        point: true,
        user: true,
      },
    });
    if (!orders) {
      throw new OrdersEmpty();
    }
    return orders;
  }

  async getAllOpenOrders(userPhone: string): Promise<PrismaOrder[]> {
    const user = await this.userService.getUser(userPhone);
    const orders = await this.prisma.order.findMany({
      where: {
        userId: user.id,
        type: 'In Progress',
      },
    });

    if (!orders) throw new OpenOrdersDoestExist();

    return orders;
  }

  async cancelOrder(
    userPhone: string,
    order_: string | number,
  ): Promise<PrismaOrder> {
    // Verifying if the user exists
    const user = await this.userService.getUser(userPhone);

    // Verifying if the order exists
    await this.getOrderByCode(userPhone, order_);
    const order = await this.prisma.order.update({
      where: {
        code: Number(order_),
        userId: user.id,
      },
      data: {
        type: 'Canceled',
      },
    });
    return order;
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

  async updateOrderStatusByCode(orderCode: number, status: string) {
    await this.ensureOrderExists(orderCode);

    const orderUpdated = await this.prisma.order.update({
      where: { code: orderCode },
      data: { type: status },
    });

    return orderUpdated;
  }

  private async ensureOrderExists(orderCode: number) {
    const order = await this.prisma.order.findUnique({
      where: { code: orderCode },
    });
    if (!order) {
      throw new OrderDoesntExists();
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

  async getQueue(): Promise<any> {
    try {
      const orders = await this.prisma.order.findMany({
        where: {
          type: {
            in: ['In Progress', 'Collecting', 'To confirm'],
          },
        },
        include: {
          tool: true,
          point: true,
          user: true,
        },
      });
      return orders;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    }
  }

  async getHistory(): Promise<any> {
    try {
      const orders = await this.prisma.order.findMany({
        where: {
          type: {
            in: ['Finished', 'Canceled'],
          },
        },
        include: {
          tool: true,
          point: true,
          user: true,
        },
      });
      return orders;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    }
  }
}
