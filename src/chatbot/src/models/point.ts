import { PrismaClient, Point as PrismaPoint } from "@prisma/client";
import { Message } from "whatsapp-web.js";

export default class PointService {
	constructor(private prisma: PrismaClient) {
		this.prisma = prisma;
	}

	async getPoint(name: string): Promise<PrismaPoint | null> {
		try {
			const point = await this.prisma.point.findFirst({
				where: {
					name: name
				}
			});
			if (point == null) {
				return null;
			}
			return point;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}

	async createPoint(point: PrismaPoint): Promise<PrismaPoint | null> {
		try {
			const newPoint = await this.prisma.point.create({data: point});
			if (newPoint == null) {
				return null;
			}
			return point;
		} catch (error) {
			console.error("An error occurred while fetching the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}
}
