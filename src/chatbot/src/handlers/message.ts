import { PrismaClient, User as PrismaUser } from "@prisma/client";
import UserService from "../models/user";
import { Client, Message, List } from "whatsapp-web.js";
import * as cli from "../cli/ui";
// Config & Constants
import config from "../config";

import {handleCreateUser, handleUpdateUser} from "../messages/user/user-messages"

// // Speech API & Whisper
import { TranscriptionMode } from "../types/transcription-mode";
import { transcribeRequest } from "../providers/speech";
import { transcribeAudioLocal } from "../providers/whisper-local";
import { transcribeWhisperApi } from "../providers/whisper-api";

const { v4: uuidv4 } = require("uuid");

// Define interfaces for the Command and Chain of Responsibility patterns
interface IRequestStateHandler {
    handle(requestState: number, message: Message, userName: string): Promise<void>;
}

interface IRequestAccess {
    handle(message: Message, user: PrismaUser | null): Promise<void>;
}

interface IMessageValidator {
    validate(message: Message): Promise<boolean>;
}

// Command Pattern for handling different request states
class RequestStateHandler implements IRequestStateHandler {
    private whatsappClient: Client;
    private userService: UserService;

    constructor(whatsappClient: Client, userService: UserService) {
        this.whatsappClient = whatsappClient;
        this.userService = userService;
    }

    async handle(requestState: number, message: Message, userName: string): Promise<void> {
        // Implement logic based on requestState
    }
}

class RequestAccess implements IRequestAccess {
    private whatsappClient: Client;
    private userService: UserService;

    constructor(whatsappClient: Client, userService: UserService) {
        this.whatsappClient = whatsappClient;
        this.userService = userService;
    }

    async handle(message: Message, user: PrismaUser | null): Promise<void> {
		if(user?.name == ""){
            handleUpdateUser(message, this.whatsappClient)
		}
        if(user?.name != ""){
            handleLeadAcess(message, this.whatsappClient)
        }
        else{
            handleCreateUser(message, this.whatsappClient)
        }
    }
}

// Base class for Chain of Responsibility Pattern
abstract class MessageValidator implements IMessageValidator {
    protected nextValidator?: IMessageValidator;

    constructor(nextValidator?: IMessageValidator) {
        this.nextValidator = nextValidator;
    }

    async validate(message: Message): Promise<boolean> {
        if (this.nextValidator) {
            return this.nextValidator.validate(message);
        }
        return true; // If all validations pass
    }
}

// Specific validator for group messages
class GroupMessageValidator extends MessageValidator {
    async validate(message: Message): Promise<boolean> {
        if ((await message.getChat()).isGroup) {
            return false; // Group messages are ignored
        }
        return super.validate(message);
    }
}

// Specific validator for checking bot readiness
class BotReadyValidator extends MessageValidator {
    validate(message: Message): Promise<boolean> {
        if (botReadyTimestamp == null || new Date(message.timestamp * 1000) < botReadyTimestamp) {
            return Promise.resolve(false); // Bot not ready or old message
        }
        return super.validate(message);
    }
}

// Your MessageEventHandler class
export class MessageEventHandler {
	private prismaClient: PrismaClient;
    private userService: UserService;
    private whatsappClient: Client;
    private messageValidator: IMessageValidator;

    constructor(prismaClient: PrismaClient, userService: UserService, whatsappClient: Client) {
        this.userService = userService;
        this.whatsappClient = whatsappClient;
        this.messageValidator = new BotReadyValidator(new GroupMessageValidator());
    }

    async handleIncomingMessage(message: Message): Promise<void> {
        if (!(await this.messageValidator.validate(message))) {
            return; // Validation failed
        }

        const userData = await this.userService.getUser(message.from);

        if (userData?.role?.includes("USER") || userData?.role?.includes("ADMIN")) {
            let requestState = userData?.requestState;
            const requestStateHandler = new RequestStateHandler(this.whatsappClient, this.userService);
            requestStateHandler.handle(requestState, message, userData.name);
        } 
        if(userData?.role?.includes("LEAD") || userData == null) {
            let requestAccess = new RequestAccess(this.whatsappClient, this.userService)
			requestAccess.handle(message, userData)
        }
    }

    // ... Rest of the class
}

let botReadyTimestamp: Date | null = new Date();


function handleLeadAcess(message: Message, whatsappClient: Client) {
    throw new Error("Function not implemented.");
}

