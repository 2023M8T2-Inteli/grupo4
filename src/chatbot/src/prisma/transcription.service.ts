import { Inject, Injectable } from '@nestjs/common';
import { PrismaService } from './prisma.service';

@Injectable()
export class TranscriptionService {
  constructor(@Inject(PrismaService) private prisma: PrismaService) {}

  async getTranscriptionFromMsgId(msgId: string): Promise<string | null> {
    const res = await this.prisma.transcription.findFirst({
      where: {
        messageId: msgId,
      },
    });
    if (!res || !res?.transcription) return null;
    return res.transcription;
  }

  async insertNewTranscription(
    messageId: string,
    mediaId: string,
    transcription: string,
  ) {
    return this.prisma.transcription.create({
      data: {
        transcription,
        mediaId,
        messageId,
      },
    });
  }
}