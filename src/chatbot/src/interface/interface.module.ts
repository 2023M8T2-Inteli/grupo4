import { Module } from '@nestjs/common';
import { InterfaceController } from './interface.controller';
import { InterfaceService } from './interface.service';
import { PrismaModule } from 'src/prisma/prisma.module';

@Module({
  controllers: [InterfaceController],
  providers: [InterfaceService],
  imports: [PrismaModule],
})
export class InterfaceModule {}
