import { Inject, Injectable } from '@nestjs/common';
import { HandleUserService } from './handle-user.service';
import {
  NothingToUpdate,
  UserDoesntExists,
  UserService,
} from '../prisma/user.service';
import { OrderService } from '../prisma/order.service';
import { LocationService } from '../prisma/location.service';
import { ToolService } from '../prisma/tool.service';
import { Role, Prisma } from '@prisma/client';

interface AuthorizeUserArgs {
  targetPhone?: string;
  targetName?: string;
  targetRole: Role;
}

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

  async handleAuthorizeUser(userPhone: string, args: AuthorizeUserArgs) {
    const { targetPhone, targetRole } = args;

    if (!targetPhone || !targetRole)
      return `Ops, parece que há uma inconsistência no seu pedido, está faltando as seguintes informações:
      ${!targetPhone && '\n - telefone do usuário'}
      ${!targetRole && '\n - permissão que você quer dar ao usuário'}
      \n Gostaria de tentar novamente?
      `;

    const permissionResult = await this.checkPermission(userPhone, Role.ADMIN);

    if (permissionResult) return permissionResult;

    return (
      (await this.checkPermission(userPhone, Role.ADMIN)) ||
      (await this.authorizeByPhone(targetPhone, targetRole))
    );
  }

  async handleCreateNewLocation(userPhone: string, args: any) {
    try {
      const { locationName, pointX, pointY } = args;

      if (!locationName || !pointX || !pointY)
        return `Ops, parece que há uma inconsistência no seu pedido, está faltando as seguintes informações:
      ${!locationName && '\n - Nome da localização'}
      ${!pointX && '\n - Ponto X'}
      ${!pointY && '\n - Ponto Y'}
      \n Gostaria de tentar novamente?
      `;

      return (
        (await this.checkPermission(userPhone, Role.ADMIN)) ||
        (await this.locationService.createLocation(
          locationName,
          pointX,
          pointY,
        ))
      );
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que você ainda não tem um cadastro conosco. Você gostaria de se cadastrar?';

      if (e instanceof Prisma.PrismaClientKnownRequestError)
        return 'Ops, parece que já existe uma localização com esse nome, gostaria de tentar com outro nome?';

      console.log(`Error: ${e}`);
      return 'Parece que houve um erro no sistema, contate um administrador';
    }
  }

  private async checkPermission(userPhone: string, permissionLevel: Role) {
    try {
      if ((await this.userService.getUserRole(userPhone)) !== permissionLevel)
        return 'Ops, parece que houve um erro no sistema e você não tem permissão para essa ação 🥲. Gostaria de fazer outra solicitação?';
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que você ainda não tem um cadastro conosco. Você gostaria de se cadastrar?';
    }
    return null;
  }

  private async authorizeByPhone(
    targetPhone: string,
    targetRole: 'ADMIN' | 'USER' | 'LEAD',
  ) {
    try {
      if ((await this.userService.getUserRole(targetPhone)) === targetRole)
        return 'Ops, parece que esse usuário já tem a permissão que você quer dar.';

      await this.userService.updateUserData({
        cellPhone: targetPhone,
        role: targetRole,
      });
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que um usuário com esses dados não existe no nosso sistema. Gostaria de fazer um novo pedido?';

      if (e instanceof NothingToUpdate)
        return 'Ops, parece que faltaram algumas informações para procedermos com esse pedido, poderia repití-lo por favor?';

      console.log(`Error: ${e}`);
      return 'Ops, parece que houve um problema no sistema, por favor contate um administrador.';
    }
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
