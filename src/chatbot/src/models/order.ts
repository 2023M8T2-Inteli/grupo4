import { PrismaClient, Order as PrismaOrder, Role } from "@prisma/client";
import { Message } from "whatsapp-web.js";

export default class OrderService {
	constructor(private prisma: PrismaClient) {
		this.prisma = prisma;
	}

	async getOrder(message: Message): Promise<PrismaOrder | null> {
		try {
			const user = await this.prisma.user.findFirst({where: {cellPhone: message.from}})
			if (user){
				const order = await this.prisma.order.findFirst({
					where: {
						code: Number(message.body),
						userId: user.id
					}
				});
				if (order == null) {
					return null;
				}
				return order;

			}else{
				return null;
			}
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}

	async cancelOrder(message: Message): Promise<PrismaOrder | null> {
		try {
			const user = await this.prisma.user.findFirst({where: {cellPhone: message.from}})
			if (user){
			const order = await this.prisma.order.update({
				where: {
					code: Number(message.body),
					userId: user.id
				},
				 data: {
					type: "Canled"
				}
			});
			if (order == null) {
				return null;
			}
			return order;
		}else{
			return null;}
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}

}
