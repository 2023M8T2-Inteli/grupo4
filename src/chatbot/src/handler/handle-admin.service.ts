import { forwardRef, Inject, Injectable } from '@nestjs/common';
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
import { AIService } from '../AI/AI.service';
import { WhatsappService } from '../whatsapp/whatsapp.service';
import { TranscriptionService } from '../prisma/transcription.service';
import { WebsocketService } from 'src/websockets/websocket.service';

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
    @Inject(AIService) protected aiService: AIService,
    @Inject(forwardRef(() => WhatsappService))
    protected whatsappService: WhatsappService,
    @Inject(TranscriptionService)
    protected transcriptionService: TranscriptionService,
    @Inject(WebsocketService) protected websocketService: WebsocketService,
  ) {
    super(
      userService,
      orderService,
      locationService,
      toolService,
      aiService,
      whatsappService,
      transcriptionService,
      websocketService,
    );
  }

  async handleAuthorizeUser(userPhone: string, args: AuthorizeUserArgs) {
    const { targetPhone, targetRole } = args;

    if (!targetPhone || !targetRole)
      return `Ops, parece que há uma inconsistência no seu pedido, está faltando as seguintes informações:
      ${!targetPhone && '\n - telefone do usuário'}
      ${!targetRole && '\n - permissão que você quer dar ao usuário'}
      \n Gostaria de tentar novamente?
      `;

    const formattedPhone = this.formatPhone(targetPhone);

    const permissionResult = await this.checkPermission(userPhone, Role.ADMIN);

    if (permissionResult) return permissionResult;

    return (
      (await this.checkPermission(userPhone, Role.ADMIN)) ||
      (await this.authorizeByPhone(formattedPhone, targetRole))
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
        (await this.messageLocationCreation(locationName, pointX, pointY))
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

  async handleCreateNewTool(userPhone: string, args: any) {
    try {
      const {
        toolName,
        toolPrice,
        toolTag,
        toolPointX,
        toolPointY,
        toolMinQuantity,
        toolMaxQuantity,
      } = args;

      if (
        !toolName ||
        !toolPrice ||
        !toolTag ||
        !toolPointX ||
        !toolPointY ||
        !toolMinQuantity ||
        !toolMaxQuantity
      )
        return `Ops, parece que há uma inconsistência no seu pedido, está faltando as seguintes informações:
      ${!toolName && '\n - Nome da ferramenta'}
      ${!toolPrice && '\n - Preço da ferramenta'}
      ${!toolTag && '\n - Tag da ferramenta'}
      ${!toolPointX && '\n - Ponto X da ferramenta'}
      ${!toolPointY && '\n - Ponto Y da ferramenta'}
      ${!toolMinQuantity && '\n - Quantidade mínima da ferramenta'}
      ${!toolMaxQuantity && '\n - Quantidade máxima da ferramenta'}
      \n Gostaria de tentar novamente?
      `;

      return (
        (await this.checkPermission(userPhone, Role.ADMIN)) ||
        (await this.messageToolCreation(
          toolName,
          toolPrice,
          toolTag,
          toolPointX,
          toolPointY,
          toolMinQuantity,
          toolMaxQuantity,
        ))
      );
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que você ainda não tem um cadastro conosco. Você gostaria de se cadastrar?';

      if (e instanceof Prisma.PrismaClientKnownRequestError)
        return 'Ops, parece que houve um erro ao salvar essa ferramenta. Gostaria de tentar novamente?';

      console.log(`Error: ${e}`);
      return 'Parece que houve um erro no sistema, contate um administrador';
    }
  }

  async handleEmergencyStop(userPhone: string, args: { stop: boolean }) {
    const permissionMessage = await this.checkPermission(userPhone, Role.ADMIN);
    if (permissionMessage) return permissionMessage;

    const { stop } = args;

    if (typeof stop !== 'boolean') {
      return 'Ops, parece que houve um problema no sistema. Consegue confirmar o seu pedido?';
    }

    const data = { emergency_stop: stop ? 1 : 0 };

    this.websocketService.emergencyStop(data);

    return `O sistema foi ${stop ? 'parado' : 'reiniciado'} com sucesso!`;
  }

  private async messageToolCreation(
    toolName: string,
    toolPrice: number,
    toolTag: string,
    toolPointX: number,
    toolPointY: number,
    toolMinQuantity: number,
    toolMaxQuantity: number,
  ) {
    const tool = await this.toolService.createTool(
      toolName,
      toolPrice,
      toolTag,
      toolPointX,
      toolPointY,
      toolMinQuantity,
      toolMaxQuantity,
    );
    return `Ferramenta criada com sucesso! Seguem os dados:
    \n Nome: ${tool.name}
    \n Code: ${tool.code}
    \n Preço: ${tool.price}
    \n Tag: ${tool.tag}
    \n Ponto X: ${tool.pointX}
    \n Ponto Y: ${tool.pointY}
    \n Quantidade mínima: ${tool.minQuantity}
    \n Quantidade máxima: ${tool.maxQuantity}
    `;
  }

  private async messageLocationCreation(
    locationName: string,
    pointX: number,
    pointY: number,
  ) {
    const location = await this.locationService.createLocation(
      locationName,
      pointX,
      pointY,
    );
    return `Localização criada com sucesso! Seguem os dados:
    \n Nome: ${location.name}
    \n Ponto X: ${location.pointX}
    \n Ponto Y: ${location.pointY}
    `;
  }

  private async authorizeByPhone(
    targetPhone: string,
    targetRole: 'ADMIN' | 'USER' | 'LEAD',
  ) {
    try {
      if ((await this.userService.getUserRole(targetPhone)) === targetRole)
        return 'Ops, parece que esse usuário já tem a permissão que você quer dar.';

      const user = await this.userService.updateUserData({
        cellPhone: targetPhone,
        role: targetRole,
      });

      if (user)
        return `Permissão do ${user.name} alterada para ${targetRole} com sucesso!`;
    } catch (e) {
      if (e instanceof UserDoesntExists)
        return 'Ops, parece que um usuário com esses dados não existe no nosso sistema. Gostaria de fazer um novo pedido?';

      if (e instanceof NothingToUpdate)
        return 'Ops, parece que faltaram algumas informações para procedermos com esse pedido, poderia repití-lo por favor?';

      console.log(`Error: ${e}`);
      return 'Ops, parece que houve um problema no sistema, por favor contate um administrador.';
    }
  }

  private formatPhone(phone: string) {
    phone = phone.replace(/^[^\d]+/, '');

    if (!phone.startsWith('55')) {
      phone = '55' + phone;
    }

    if (phone.endsWith('@c.us')) {
      phone = phone.slice(0, -5);
    }

    if (phone.length === 13) {
      phone = phone.slice(0, 4) + phone.slice(5);
    }

    phone += '@c.us';

    console.log(`Formatted phone: ${phone}`);

    return phone;
  }
}
