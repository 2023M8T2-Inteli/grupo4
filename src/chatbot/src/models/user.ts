import { PrismaClient, User as PrismaUser } from "@prisma/client";

interface User {
    id: string;
    name: string;
    cellPhone: string;
    requestState: number;
    createdAt: Date;
}

export default class UserService {
	
    constructor(private prisma: PrismaClient) {
        this.prisma = prisma;
    }

    async getUser(cellPhone: string): Promise<PrismaUser | null> {
        try {
            const user = await this.prisma.user.findFirst({
                where: {
                    cellPhone,
                },
            });
            if (user == null) {
                return null;
            }
            return user;
        } catch (error) {
            console.error("An error occurred while fetching the user:", error);
            throw error;
        } finally {
            await this.prisma.$disconnect();
        }
    }

    async createAccountUser(user: User): Promise<PrismaUser> {
        try {
            const createdUser = await this.prisma.user.create({
                data: {
                    ...user,
                },
            });
            return createdUser;
        } catch (error) {
            console.error("An error occurred while creating the user:", error);
            throw error;
        } finally {
            await this.prisma.$disconnect();
        }
    }

    async updateUser(user: User): Promise<PrismaUser> {
        try {
            const updatedUsers = await this.prisma.user.update({
                where: {
                    cellPhone: user.cellPhone,
                },
                data: {
                    name: user.name,
                },
            });
            return updatedUsers;
        } catch (error) {
            console.error("An error occurred while updating the user:", error);
            throw error;
        } finally {
            await this.prisma.$disconnect();
        }
    }

    async checkUserByCellPhone(cellPhone: string): Promise<boolean> {
        try {
            const user = await this.prisma.user.findFirst({
                where: {
                    cellPhone,
                },
            });
            return user != null;
        } catch (error) {
            console.error("An error occurred while fetching the user:", error);
            throw error;
        } finally {
            await this.prisma.$disconnect();
        }
    }
}