import { PrismaClient, Order as PrismaOrder, Role } from "@prisma/client";

interface Order {
	id: string;
	name: string;
	pryce: string;
	quantity: number;
	createdAt: Date;
}

export default class OrderService {
	constructor(private prisma: PrismaClient) {
		this.prisma = prisma;
	}

	async getOrder(id: string): Promise<PrismaOrder | null> {
		try {
			const order = await this.prisma.order.findFirst({
				where: {
					id
				}
			});
			if (order == null) {
				return null;
			}
			return order;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}

}
