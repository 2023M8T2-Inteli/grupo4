import { PrismaClient, User as PrismaUser, Role } from "@prisma/client";

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
					cellPhone
				}
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

	async getAdmin(): Promise<PrismaUser | null> {
		try {
			const user = await this.prisma.user.findFirst({
				where: {
					role: {
						equals: [Role.ADMIN]
					}
				}
			});
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
					...user
				}
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
			const updatedUser = await this.prisma.user.update({
				where: {
					cellPhone: user.cellPhone
				},
				data: {
					name: user.name
				}
			});
			return updatedUser;
		} catch (error) {
			console.error("An error occurred while updating the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}

	async updateRequestUser(cellPhone: string, requestState: number): Promise<PrismaUser> {
		try {
			const resquestUser = await this.prisma.user.update({
				where: {
					cellPhone: cellPhone
				},
				data: {
					requestState: requestState
				}
			});

			return resquestUser;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}
}
