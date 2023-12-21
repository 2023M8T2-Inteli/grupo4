import { Inject, Injectable } from '@nestjs/common';
import { PrismaService } from './prisma.service';
import { GPTFunctionCalling as GPTFunctionCallingPrisma } from '@prisma/client';

@Injectable()
export class GPTFunctionCallingService {
  constructor(@Inject(PrismaService) private prisma: PrismaService) {}

  async insertNewFunctionCalling(
    functionId: string,
    functionCallObject: string,
    functionName: string,
    messageId: string,
    userId: string,
  ) {
    return this.prisma.gPTFunctionCalling.create({
      data: {
        id: functionId,
        functionCallObject,
        functionName,
        messageId,
        userId,
      },
    });
  }

  async addFunctionCallingResponse(messageId: string, response: string) {
    const { id } = await this.prisma.gPTFunctionCalling.findFirst({
      where: {
        messageId: messageId,
      },
    });

    return this.prisma.gPTFunctionCalling.update({
      where: {
        id,
      },
      data: {
        functionResponse: response,
      },
    });
  }

  async getFunctionCallingByUserId(userId: string) {
    const timeLimit = new Date(Math.floor(+new Date() / 1000) - 86400);

    return await this.prisma.gPTFunctionCalling.findMany({
      where: {
        userId: userId,
        createdAt: {
          gt: timeLimit,
        },
      },
    });
  }
}
