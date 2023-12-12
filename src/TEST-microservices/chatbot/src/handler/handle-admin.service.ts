import { Inject, Injectable } from '@nestjs/common';
import UserService from 'src/prisma/user.service';
import { WhatsappService } from 'src/whatsapp/whatsapp.service';

@Injectable()
export class handleAdminService {
  constructor(
    @Inject(UserService) private userService: UserService,
    @Inject(WhatsappService) private whatsappService: WhatsappService,
  ) {}

  async handle(requestState: number, message: Message): Promise<void> {
    switch (requestState) {
      case 1:
        handleAdminRequestMenu(message, this.whatsappClient);
        break;
      case 2:
        handleAdminProcessRequest(message, this.whatsappClient);
        break;
      case 3:
        handleNewPoint(message, this.whatsappClient);
      case 4:
        handleUpdateUserAccess(message, this.whatsappClient);
      default:
        break;
    }
  }
}
