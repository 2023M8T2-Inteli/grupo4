import { PrismaClient, User as PrismaUser, Role } from "@prisma/client";
import { Message } from "whatsapp-web.js";

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

	async getUserbyId(id: string): Promise<PrismaUser | null> {
		try {
			const user = await this.prisma.user.findFirst({
				where: {
					id: id
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
						equals: ["ADMIN"]
					}
				}
			});
			if (user) {
				return user;
			}
			return user;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}

	async createAccountUser(user: PrismaUser): Promise<PrismaUser> {
		try {
			const createdUser = await this.prisma.user.create({
				data: user
			});
			return createdUser;
		} catch (error) {
			console.error("An error occurred while creating the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}

	async updateAccountUser(user: PrismaUser): Promise<PrismaUser> {
		try {
			const updatedUser = await this.prisma.user.update({
				where: {
					cellPhone: user.cellPhone
				},
				data: {
					name: user.name,
					requestState: user.requestState
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

	async updateVoice(user: PrismaUser): Promise<PrismaUser> {
		try {
			const updatedUser = await this.prisma.user.update({
				where: {
					cellPhone: user.cellPhone
				},
				data: {
					voice: user.voice
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

	async updateSpeedVoice(user: PrismaUser): Promise<PrismaUser> {
		try {
			const updatedUser = await this.prisma.user.update({
				where: {
					cellPhone: user.cellPhone
				},
				data: {
					speedVoice: user.speedVoice
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

	async updateName(cellPhone: string, name: string): Promise<PrismaUser> {
		try {
			const updatedUser = await this.prisma.user.update({
				where: {
					cellPhone: cellPhone
				},
				data: {
					name: name
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

	async updateRoletUser(cellPhone: string): Promise<PrismaUser> {
		try {
			const resquestUser = await this.prisma.user.update({
				where: {
					cellPhone: cellPhone
				},
				data: {
					role: [Role.USER]
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
