import { Inject, Injectable } from '@nestjs/common';
import { HandleUserService } from './handle-user.service';
import { UserService } from '../prisma/user.service';
import { OrderService } from '../prisma/order.service';
import { LocationService } from '../prisma/location.service';
import { ToolService } from '../prisma/tool.service';

@Injectable()
export class HandleAdminService extends HandleUserService {
  constructor(
    @Inject(UserService) protected userService: UserService,
    @Inject(OrderService) protected orderService: OrderService,
    @Inject(LocationService) protected locationService: LocationService,
    @Inject(ToolService) protected toolService: ToolService,
  ) {
    super(userService, orderService, locationService, toolService);
  }
}

// @Injectable()
// export class handleAdminService {
//   constructor(
//     @Inject(UserService) private userService: UserService,
//     @Inject(WhatsappService) private whatsappService: WhatsappService,
//   ) {}
//
//   async handle(requestState: number, message: Message): Promise<void> {
//     switch (requestState) {
//       case 1:
//         handleAdminRequestMenu(message, this.whatsappClient);
//         break;
//       case 2:
//         handleAdminProcessRequest(message, this.whatsappClient);
//         break;
//       case 3:
//         handleNewPoint(message, this.whatsappClient);
//       case 4:
//         handleUpdateUserAccess(message, this.whatsappClient);
//       default:
//         break;
//     }
//   }
// }
