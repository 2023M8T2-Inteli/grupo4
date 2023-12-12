import { Inject, Injectable } from '@nestjs/common';
import UserService from 'src/prisma/user.service';
import { WhatsappService } from 'src/whatsapp/whatsapp.service';

@Injectable()
export class handleLeadService {
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(WhatsappService) private whatsappService: WhatsappService,
  ) {}

  async handle(message: Message, user: PrismaUser | null): Promise<void> {
    if (user == null) {
      handleCreateUser(message, this.whatsappClient);
    }
    if (user?.name == '' && user != null) {
      handleUpdateUser(message, this.whatsappClient);
    }
    if (
      user?.name != '' &&
      user != null &&
      user?.voice != '' &&
      user?.speedVoice != 0.0
    ) {
      handleLeadAcess(message, this.whatsappClient);
    }
    if (
      user?.name != '' &&
      user != null &&
      user?.voice == '' &&
      user?.speedVoice == 0.0
    ) {
      handleUpdateUserVoice(message, this.whatsappClient);
    }
    if (
      user?.name != '' &&
      user != null &&
      user?.voice != '' &&
      user?.speedVoice == 0.0
    ) {
      handleUpdateUserSpeedVoice(message, this.whatsappClient);
    }
  }
}
