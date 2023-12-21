import { Inject, Injectable } from '@nestjs/common';
import { LocationService } from 'src/prisma/location.service';
import { Point, User } from '@prisma/client';
import { ToolService } from 'src/prisma/tool.service';
import { UserService } from 'src/prisma/user.service';
import { OrderService } from 'src/prisma/order.service';
import { AIService } from 'src/AI/AI.service';

@Injectable()
export class InterfaceService {
  private conversationHistory: any[] = [];
  constructor(
    @Inject(LocationService) private readonly locationService: LocationService,
    @Inject(ToolService)
    private readonly toolService: ToolService,
    @Inject(UserService) private readonly userService: UserService,
    @Inject(OrderService) private readonly orderService: OrderService,
    @Inject(AIService) private readonly aiService: AIService,
  ) {}

  async getAllPoints(): Promise<Point[]> {
    const locations = await this.locationService.getAllLocations();
    return locations;
  }
  async getPointById(id: string): Promise<Point> {
    const location = await this.locationService.getLocationById(id);
    return location;
  }
  async createPoint(body: any): Promise<Point> {
    let { name, pointX, pointY } = body;
    pointX = Number(pointX);
    pointY = Number(pointY);
    const location = await this.locationService.createLocation(
      name,
      pointX,
      pointY,
    );
    return location;
  }
  async deletePoint(id: string): Promise<Point> {
    const location = await this.locationService.deleteLocation(id);
    return location;
  }

  async updatePoint(id: string, body: any): Promise<Point> {
    let { name, pointX, pointY } = body;
    pointX = Number(pointX);
    pointY = Number(pointY);
    const location = await this.locationService.updateLocation(
      id,
      name,
      pointX,
      pointY,
    );
    return location;
  }

  async getAllTools(): Promise<Point[]> {
    const tools = await this.toolService.getAllTools();

    return tools;
  }

  async getToolById(id: string): Promise<Point> {
    const tool = await this.toolService.getToolById(id);
    return tool;
  }

  async createTool(body: any): Promise<Point> {
    let { name, price, tag, pointX, pointY, minQuantity, maxQuantity } = body;
    price = Number(price);
    pointX = Number(pointX);
    pointY = Number(pointY);
    minQuantity = Number(minQuantity);
    maxQuantity = Number(maxQuantity);
    const location = await this.toolService.createTool(
      name,
      price,
      tag,
      pointX,
      pointY,
      minQuantity,
      maxQuantity,
    );
    return location;
  }

  async deleteTool(id: string): Promise<Point> {
    const tool = await this.toolService.deleteTool(id);
    return tool;
  }

  async updateTool(id: string, body: any): Promise<Point> {
    let { name, price, tag, pointX, pointY, minQuantity, maxQuantity } = body;
    price = Number(price);
    if (pointX == undefined || pointY == undefined) {
      pointX = 0.0;
      pointY = 0.0;
    } else {
      pointX = Number(pointX);
      pointY = Number(pointY);
    }
    minQuantity = Number(minQuantity);
    maxQuantity = Number(maxQuantity);
    const location = await this.toolService.updateTool(
      id,
      name,
      price,
      tag,
      pointX,
      pointY,
      minQuantity,
      maxQuantity,
    );
    return location;
  }

  async getAllUsers(): Promise<User[]> {
    const users = await this.userService.getAllUsers();
    return users;
  }

  async getUserById(id: string): Promise<User> {
    const user = await this.userService.getUserById(id);
    return user;
  }

  async deleteUser(id: string): Promise<User> {
    const user = await this.userService.deleteUser(id);
    return user;
  }

  async updateUser(id: string, body: any): Promise<User> {
    let { name, cellPhone, role } = body;
    const user = await this.userService.updateUser(name, cellPhone, role);
    return user;
  }

  async getQueue(): Promise<any> {
    const queue = await this.orderService.getQueue();
    return queue;
  }

  async getHistory(): Promise<any> {
    // this.aiService.getMessageEmotion()
    // this.aiService.buildInterfaceSystemMessage()
    const history = await this.orderService.getHistory();
    return history;
  }

  getFilteredData(data, props) {
    const filteredData = {};
    for (const prop of props) {
      if (typeof prop === 'object') {
        // Nested property
        const [key, nestedProps] = Object.entries(prop)[0];
        if (data?.key && typeof data[key] === 'object') {
          filteredData[key] = this.getFilteredData(data[key], nestedProps);
        }
      } else if (data[prop] !== undefined) {
        // Simple property
        filteredData[prop] = data[prop];
      }
    }
    return filteredData;
  }

  async speak(body: any): Promise<any> {
    let { audioData } = body;
    let transcript = await this.aiService.speech2Text(audioData);
    this.conversationHistory.push({ role: 'user', content: transcript });
    let system = await this.aiService.buildInterfaceSystemMessage();
    console.log(system)
    let response = await this.aiService.callGPTInterface(
      system,
      this.conversationHistory,
    );
    this.conversationHistory.push({ role: 'system', content: response });
    let emotion = await this.aiService.getMessageEmotion(response);
    let audio = await this.aiService.text2Speech(response);
    return {
      message: 'Audio file received, transcribed, and processed successfully',
      base64Audio: audio,
      emotion,
    };
  }

}
