import { Inject, Injectable } from '@nestjs/common';
import {
  UserAlreadyExists,
  UserDoesntExists,
  UserService,
} from '../prisma/user.service';
import { WhatsappService } from 'src/whatsapp/whatsapp.service';

interface CreateUserArgs {
  firstName: string;
  lastName: string;
}

@Injectable()
export class HandleLeadService {
  constructor(@Inject(UserService) private userService: UserService, @Inject(WhatsappService) private readonly whatsappService: WhatsappService) {}
  async handleCreateUser(userPhone: string, args: CreateUserArgs) {
    const firstName = args?.firstName || '';
    const lastName = args?.lastName || '';

    if ( firstName && lastName)
      try {
        await this.userService.createAccountUser({
          name: firstName + ' ' + lastName,
          cellPhone: userPhone,
        });
        return 'Sua conta foi criada! 😀 Agora, você deve aguardar que um administrador libere seu acesso!';
      } catch (e) {
        if (e instanceof UserAlreadyExists) {
          return 'Parece que você já está cadastrado em nosso sistema, aguarde que um administrador lhe contatará.';
        }
        console.log(`Error: ${e}`);
        return 'Ocorreu um erro ao criar sua conta, por favor contate um administrador.';
      }

    return `Preciso de mais algumas informações! Me envie: ${
      firstName ? '' : '\n - primeiro nome,'
    }
    ${lastName ? '' : '\n -sobrenome,'} \n  😀`;
  }

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  async handleLeadAccess(userPhone: string, _args: object) {
    try {
      const user = await this.userService.getUser(userPhone);

      if (user.role != 'LEAD') {
        return 'Parece que você acabou de ganhar um up no nosso sistema! Em que posso lhe ajudar?';
      } else {
        this.whatsappService.sendMessage(userPhone, 'Opa! Encontrei o seu cadastro aqui, mas você ainda não está com permissões de acessar nosso serviço!');
        const adminContact = await this.whatsappService.getAdminContact()
        this.whatsappService.sendMessage(userPhone, adminContact);
        return "Você pode entrar com a pessoa acima ou aguardar que um administrador libere seu acesso 😀"
      }
    } catch (e) {
      if (e instanceof UserDoesntExists) {
        return 'Parece que você ainda não esta cadastrado em nosso sistema. Você gostaria de fazer o cadastro agora?';
      } else {
        console.log(`Error: ${e}`);
        return 'Não foi possível consultar a sua situação, por favor consulte um administrador.';
      }
    }
  }
}

// @Injectable()
// export class handleLeadService {
//   constructor(
//     @Inject(UserService) private userService: UserService,
//     @Inject(WhatsappService) private whatsappService: WhatsappService,
//   ) {}
//
//   async handle(message: Message, user: PrismaUser | null): Promise<void> {
//     if (user == null) {
//       handleCreateUser(message, this.whatsappClient);
//     }
//     if (user?.name == '' && user != null) {
//       handleUpdateUser(message, this.whatsappClient);
//     }
//     if (
//       user?.name != '' &&
//       user != null &&
//       user?.voice != '' &&
//       user?.speedVoice != 0.0
//     ) {
//       handleLeadAcess(message, this.whatsappClient);
//     }
//     if (
//       user?.name != '' &&
//       user != null &&
//       user?.voice == '' &&
//       user?.speedVoice == 0.0
//     ) {
//       handleUpdateUserVoice(message, this.whatsappClient);
//     }
//     if (
//       user?.name != '' &&
//       user != null &&
//       user?.voice != '' &&
//       user?.speedVoice == 0.0
//     ) {
//       handleUpdateUserSpeedVoice(message, this.whatsappClient);
//     }
//   }
// }
