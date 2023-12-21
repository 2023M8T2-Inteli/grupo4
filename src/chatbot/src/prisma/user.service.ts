import { Inject, Injectable } from '@nestjs/common';
import { User as PrismaUser, Role } from '@prisma/client';
import { PrismaService } from './prisma.service';

interface UserCreationData {
  name: string;
  cellPhone: string;
}

export class UserDoesntExists extends Error {
  constructor(message: string = 'User doesnt exists') {
    super(message);
    this.name = 'UserDoesntExists';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, UserDoesntExists);
    }
  }
}

export class UserAlreadyExists extends Error {
  constructor(message: string = 'User already exists') {
    super(message);
    this.name = 'UserAlreadyExists';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, UserAlreadyExists);
    }
  }
}

export class NothingToUpdate extends Error {
  constructor(message: string = 'Nothing to update') {
    super(message);
    this.name = 'NothingToUpdate';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, NothingToUpdate);
    }
  }
}

@Injectable()
export class UserService {
  constructor(@Inject(PrismaService) private prisma: PrismaService) {}

  async getUser(cellPhone: string): Promise<PrismaUser | null> {
    const user = await this.prisma.user.findFirst({
      where: {
        cellPhone,
      },
    });
    if (user == null) {
      throw new UserDoesntExists();
    }
    return user;
  }

  async getUserById(id: string): Promise<PrismaUser> {
    return await this.prisma.user.findUniqueOrThrow({
      where: {
        id,
      },
    });
  }

  async getUserRole(cellPhone: string): Promise<Role> {
    const user = await this.getUser(cellPhone);
    return user.role;
  }

  async getAdmin(): Promise<PrismaUser | null> {
    try {
      const user = await this.prisma.user.findFirst({
        where: {
          role: {
            equals: 'ADMIN',
          },
        },
      });
      if (user) {
        return user;
      }
      return user;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async createAccountUser(user: UserCreationData): Promise<PrismaUser> {
    const tempUser = await this.prisma.user.findFirst({
      where: {
        cellPhone: {
          equals: user.cellPhone,
        },
      },
    });
    if (tempUser) {
      throw new UserAlreadyExists();
    }

    try {
      const createdUser = await this.prisma.user.create({
        data: {
          ...user,
          voice: 'DEFAULT',
          speedVoice: 1.0,
        },
      });
      return createdUser;
    } catch (error) {
      console.error('An error occurred while creating the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateUserData(user: Partial<PrismaUser>): Promise<PrismaUser> {
    if (user.cellPhone) await this.getUser(user.cellPhone);
    else throw new UserDoesntExists();

    if (user.id) delete user.id;

    if (!this.hasAtLeastOneField(user)) throw new NothingToUpdate();
    const updatedUser = await this.prisma.user.update({
      where: {
        cellPhone: user.cellPhone,
      },
      data: user,
    });
    return updatedUser;
  }

  private hasAtLeastOneField(obj: Partial<PrismaUser>): boolean {
    return Object.values(obj).some(
      (value) => value !== undefined && value !== null,
    );
  }

  async updateAccountUser(user: PrismaUser): Promise<PrismaUser> {
    try {
      const updatedUser = await this.prisma.user.update({
        where: {
          cellPhone: user.cellPhone,
        },
        data: {
          name: user.name,
          requestState: user.requestState,
        },
      });
      return updatedUser;
    } catch (error) {
      console.error('An error occurred while updating the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateVoice(user: PrismaUser): Promise<PrismaUser> {
    try {
      const updatedUser = await this.prisma.user.update({
        where: {
          cellPhone: user.cellPhone,
        },
        data: {
          voice: user.voice,
        },
      });
      return updatedUser;
    } catch (error) {
      console.error('An error occurred while updating the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateSpeedVoice(user: PrismaUser): Promise<PrismaUser> {
    try {
      const updatedUser = await this.prisma.user.update({
        where: {
          cellPhone: user.cellPhone,
        },
        data: {
          speedVoice: user.speedVoice,
        },
      });
      return updatedUser;
    } catch (error) {
      console.error('An error occurred while updating the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateName(cellPhone: string, name: string): Promise<PrismaUser> {
    try {
      const updatedUser = await this.prisma.user.update({
        where: {
          cellPhone: cellPhone,
        },
        data: {
          name: name,
        },
      });
      return updatedUser;
    } catch (error) {
      console.error('An error occurred while updating the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateRequestUser(
    cellPhone: string,
    requestState: number,
  ): Promise<PrismaUser> {
    try {
      const resquestUser = await this.prisma.user.update({
        where: {
          cellPhone: cellPhone,
        },
        data: {
          requestState: requestState,
        },
      });

      return resquestUser;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateRoletUser(cellPhone: string): Promise<PrismaUser> {
    try {
      const resquestUser = await this.prisma.user.update({
        where: {
          cellPhone: cellPhone,
        },
        data: {
          role: Role.USER,
        },
      });

      return resquestUser;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async getAllUsers(): Promise<PrismaUser[]> {
    try {
      const users = await this.prisma.user.findMany();
      return users;
    } catch (error) {
      console.error('An error occurred while fetching the user:', error);
      throw error;
    } finally {
      await this.prisma.$disconnect();
    }
  }

  async updateUser(
    id: string,
    name?: string,
    cellPhone?: string,
    role?: Role,
): Promise<PrismaUser> {
    try {
        const data: Record<string, any> = {};

        // Only include properties in the data object if they are provided
        if (name !== undefined) data.name = name;
        if (cellPhone !== undefined) data.cellPhone = cellPhone;
        if (role !== undefined) data.role = role;

        const user = await this.prisma.user.update({
            where: {
                id: id,
            },
            data: data,
        });

        return user;
    } catch (error) {
        console.error('An error occurred while updating the user:', error);
        throw error;
    } finally {
        await this.prisma.$disconnect();
    }
}

async deleteUser(id: string): Promise<PrismaUser> {
    try {
        const user = await this.prisma.user.delete({
            where: {
                id: id,
            },
        });

        return user;
    } catch (error) {
        console.error('An error occurred while deleting the user:', error);
        throw error;
    } finally {
        await this.prisma.$disconnect();
    }
  }
}
