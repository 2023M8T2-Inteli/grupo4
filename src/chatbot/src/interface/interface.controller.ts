import {
  Body,
  Controller,
  Delete,
  Get,
  Inject,
  Param,
  Post,
  Put,
  UploadedFile,
  UseInterceptors,
  
} from '@nestjs/common';
import { InterfaceService } from './interface.service';
import { Point, User } from '@prisma/client';
import { Express } from 'express'
import {FileInterceptor} from '@nestjs/platform-express'

@Controller('')
export class InterfaceController {
  constructor(
    @Inject(InterfaceService)
    private readonly interfaceService: InterfaceService,
  ) {}
  @Get('/points')
  async getAllPoints(): Promise<Point[]> {
    return await this.interfaceService.getAllPoints();
  }
  @Get('/points/:id')
  async getPointById(@Param('id') params: any): Promise<Point> {
    return await this.interfaceService.getPointById(params);
  }
  @Post('/points')
  async createPoint(@Body() body: any): Promise<Point> {
    console.log('cheguei');
    return await this.interfaceService.createPoint(body);
  }
  @Delete('/points/:id')
  async deletePoint(@Param('id') params: any): Promise<Point> {
    return await this.interfaceService.deletePoint(params);
  }

  @Put('/points/:id')
  async updatePoint(
    @Param('id') params: any,
    @Body() body: any,
  ): Promise<Point> {
    return await this.interfaceService.updatePoint(params, body);
  }

  @Get('/tools/')
  async getAllTools(): Promise<Point[]> {
    return await this.interfaceService.getAllTools();
  }

  @Get('/tools/:id')
  async getToolById(@Param('id') params: any): Promise<Point> {
    return await this.interfaceService.getToolById(params);
  }

  @Post('/tools')
  async createTool(@Body() body: any): Promise<Point> {
    return await this.interfaceService.createTool(body);
  }

  @Delete('/tools/:id')
  async deleteTool(@Param('id') params: any): Promise<Point> {
    return await this.interfaceService.deleteTool(params);
  }

  @Put('/tools/:id')
  async updateTool(
    @Param('id') params: any,
    @Body() body: any,
  ): Promise<Point> {
    return await this.interfaceService.updateTool(params, body);
  }

  @Get('/users')
  async getAllUsers(): Promise<User[]> {
    return await this.interfaceService.getAllUsers();
  }

  @Get('/users/:id')
  async getUserById(@Param('id') params: any): Promise<User> {
    return await this.interfaceService.getUserById(params);
  }

  @Delete('/users/:id')
  async deleteUser(@Param('id') params: any): Promise<User> {
    return await this.interfaceService.deleteUser(params);
  }

  @Put('/users/:id')
  async updateUser(@Param('id') params: any, @Body() body: any): Promise<User> {
    return await this.interfaceService.updateUser(params, body);
  }

  @Get('/orders/queue')
  async getQueue(): Promise<any> {
    return await this.interfaceService.getQueue();
  }

  @Get('/orders/history')
  async getHistory(): Promise<any> {
    return await this.interfaceService.getHistory();
  }

  @Post('/speak')
    async speak(@Body() body: any): Promise<any> {
      
        return await this.interfaceService.speak(body);
    }
}
