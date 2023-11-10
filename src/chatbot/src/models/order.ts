import { PrismaClient, User as PrismaUser, AllowedUsers as PrismaAllowedUsers, Role } from "@prisma/client";

interface Order {
	id: string;
	name: string;
	pryce: string;
	quantity: number;
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

}
