import { Inject, Injectable } from '@nestjs/common';
import { Tool as PrismaTool } from '@prisma/client';
import { PrismaService } from './prisma.service';

export class ToolDoesntExists extends Error {
  constructor(message: string = 'Tool doesnt exists') {
    super(message);
    this.name = 'ToolDoesntExists';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, ToolDoesntExists);
    }
  }
}

export class TableToolIsEmpty extends Error {
  constructor(message: string = 'Table tool is empty') {
    super(message);
    this.name = 'TableToolIsEmpty';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, TableToolIsEmpty);
    }
  }
}

@Injectable()
export class ToolService {
  constructor(@Inject(PrismaService) private prisma: PrismaService) {}

  async createTool(
    name: string,
    price: number,
    tag: string,
    pointX: number,
    pointY: number,
    minQuantity: number,
    maxQuantity: number,
  ): Promise<PrismaTool> {
    const tool = await this.prisma.tool.create({
      data: {
        name,
        price,
        tag,
        pointX,
        pointY,
        pointZ: 0.0,
        minQuantity,
        maxQuantity,
      },
    });
    return tool;
  }

  async getAllTools(): Promise<PrismaTool[]> {
    const tools = await this.prisma.tool.findMany();

    if (!tools) throw new TableToolIsEmpty();

    return tools;
  }

  async getToolByCoords(coords: number[]): Promise<PrismaTool> {
    const tool = await this.prisma.tool.findFirst({
      where: {
        pointX: {
          equals: coords[0],
        },
        pointY: {
          equals: coords[1],
        },
      },
    });

    if (!tool) throw new ToolDoesntExists();

    return tool;
  }

  async coordsExists(coords: number[]): Promise<boolean> {
    try {
      await this.getToolByCoords(coords);
      return true;
    } catch {
      return false;
    }
  }

  async getToolIdByCoords(coords: number[]): Promise<string> {
    const tool = await this.getToolByCoords(coords);
    return tool.id;
  }

  async getToolById(id: string): Promise<PrismaTool> {
    const tool = await this.prisma.tool.findUnique({
      where: {
        id: id,
      },
    });

    if (!tool) throw new ToolDoesntExists();

    return tool;
  }

  async deleteTool(id: string): Promise<PrismaTool> {
    const tool = await this.prisma.tool.delete({
      where: {
        id: id,
      },
    });

    if (!tool) throw new ToolDoesntExists();

    return tool;
  }

  async updateTool(
    id: string,
    name?: string,
    price?: number,
    tag?: string,
    minQuantity?: number,
    maxQuantity?: number,
    pointX?: number,
    pointY?: number
): Promise<PrismaTool> {
    const data: Record<string, any> = {};

    // Only include properties in the data object if they are provided
    if (name !== undefined) data.name = name;
    if (price !== undefined) data.price = price;
    if (tag !== undefined) data.tag = tag;
    if (pointX !== undefined) data.pointX = pointX;
    if (pointY !== undefined) data.pointY = pointY;
    if (minQuantity !== undefined) data.minQuantity = minQuantity;
    if (maxQuantity !== undefined) data.maxQuantity = maxQuantity;

    // Assuming 'pointZ' has a default value of 0.0
    data.pointZ = 0.0;

    const tool = await this.prisma.tool.update({
        where: {
            id: id,
        },
        data: data,
    });

    if (!tool) throw new ToolDoesntExists();

    return tool;
}


}
