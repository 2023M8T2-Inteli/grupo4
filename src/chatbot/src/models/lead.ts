import { PrismaClient, Lead as PrismaLead } from "@prisma/client";

interface Lead {
	id: string;
	name: string;
	cellPhone: string;
	createdAt: Date;
}

export default class LeadService {
	constructor(private prisma: PrismaClient) {
		this.prisma = prisma;
	}

    async createAccountLead(lead: Lead): Promise<PrismaLead> {
		try {
			const newLead = await this.prisma.lead.create({
				data: {
					... lead
				}
			});
			return newLead;
		} catch (error) {
			console.error("An error occurred while creating the user:", error);
			throw error;
		} finally {
			await this.prisma.$disconnect();
		}
	}
}
