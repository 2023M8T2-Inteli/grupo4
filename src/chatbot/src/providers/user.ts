const { PrismaClient } = require("@prisma/client");
const prisma = new PrismaClient();

export default class User {
    id: string;
    name: string;
    cellPhone: string;
    requestState: number;

    constructor(id: string, name: string, cellPhone: string, requestState: number) {
        this.id = id;
        this.name = name;
        this.cellPhone = cellPhone;
        this.requestState = 0;
    }

    updateRequestState(requestState: number) {
        this.requestState = requestState;
    }

    async getUser() {
        try {
            
            const user = await prisma.user.findFirst({
                where: {
                    cellPhone: this.cellPhone
                }
            });
            prisma.$disconnect();
            return user;
        } catch (error) {
            console.error("An error occurred while fetching the user:", error);
            throw error;
        }
    }
    
    async createAccountUser() {
        try {
            const user = await prisma.user.create({
                data: {
                    id: this.id,
                    name: this.name,
                    cellPhone: this.cellPhone
                }
            });
            prisma.$disconnect();
            return user;
        } catch (error) {
            console.error("An error occurred while fetching the user:", error);
            throw error;
        }
    }

    async updateUser() {
        try {
            const user = await prisma.user.updateMany({
                where: {
                    cellPhone: this.cellPhone
                },
                data: {
                    name: this.name
                }
            });
            prisma.$disconnect();
            return user;
        } catch (error) {
            console.error("An error occurred while fetching the user:", error);
            throw error;
        }
    }
}