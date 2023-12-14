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
}
