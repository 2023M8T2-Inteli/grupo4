import { PrismaClient, Order as PrismaOrder, Role } from '@prisma/client';
import { Message } from 'whatsapp-web.js';
const { v4: uuidv4 } = require('uuid');
export default class OrderService {
  constructor(private prisma: PrismaClient) {
    this.prisma = prisma;
  }

  async getOrder(message: Message): Promise<PrismaOrder | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: message.from },
      });
      if (user != null) {
        const order = await this.prisma.order.findFirst({
          where: {
            code: Number(message.body),
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
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async createOrder(
    cellPhone: string,
    toolId: string
  ): Promise<PrismaOrder | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: cellPhone },
      });
      if (user != null) {
        const orders = await this.prisma.order.create({
          data: {
            id: uuidv4(),
            type: 'In Progress',
            toolId: toolId,
            userId: user.id,
            pointId: '123e4567-e89b-12d3-a456-426614174013',
          },
        });
        if (orders == null) {
          return null;
        }
        return orders;
      } else {
        return null;
      }
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateOrder(
    cellPhone: string,
    pointId: string
  ): Promise<Number | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: cellPhone },
      });
      if (user != null) {
        const order = await this.prisma.order.updateMany({ where: { userId: user.id, type: "In Progress", pointId: "123e4567-e89b-12d3-a456-426614174013" }, data: { pointId: pointId } });
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
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async verifyOpenOrder(message: Message): Promise<PrismaOrder | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: message.from },
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
    } finally {
      await this.prisma.$disconnect();
    }
  }
  async getOpenOrder(message: Message): Promise<PrismaOrder[] | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: message.from },
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
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async cancelOrder(message: Message): Promise<PrismaOrder | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: { cellPhone: message.from },
      });
      if (user) {
        const order = await this.prisma.order.update({
          where: {
            code: Number(message.body),
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
    } finally {
      await this.prisma.$disconnect();
    }
  }
}
