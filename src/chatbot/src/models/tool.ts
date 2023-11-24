import { PrismaClient, Tool as PrismaTool, Role } from "@prisma/client";
import { Message } from "whatsapp-web.js";

export default class ToolService {
	constructor(private prisma: PrismaClient) {
		this.prisma = prisma;
	}

	async getTool(message: Message): Promise<PrismaTool | null> {
		try {
			const tool = await this.prisma.tool.findFirst({
				where: {
					code: Number(message.body)
				}
			});
			if (tool == null) {
				return null;
			}
			return tool;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}
	async getCatalog(): Promise<PrismaTool[] | null> {
		try {
			const tools = await this.prisma.tool.findMany();
			if (tools == null) {
				return null;
			}
			return tools;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}
	async getToolsbyTag(tag: string): Promise<PrismaTool[] | null> {
		try {
			let orders;
			if (tag == null) {
				orders = await this.prisma.tool.findMany();
			} else {
				orders = await this.prisma.tool.findFirst({ where: { tag: tag } });
			}

			if (orders == null) {
				return null;
			}
			return orders;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}
}
