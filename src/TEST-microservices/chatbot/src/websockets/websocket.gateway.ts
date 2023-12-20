import {
  MessageBody,
  OnGatewayConnection,
  OnGatewayDisconnect,
  OnGatewayInit,
  SubscribeMessage,
  WebSocketGateway,
  WebSocketServer,
  WsResponse,
} from '@nestjs/websockets';
import { WebsocketService } from './websocket.service';
import { Server } from 'socket.io';
import { Inject, Injectable, forwardRef } from '@nestjs/common';

interface ItaskFeedback {
  id: string;
  type: 'GRAB' | 'DROP';
}

@Injectable()
@WebSocketGateway(3030, { transports: ['polling', 'websocket'], cors: true })
export class WebsocketGateway
  implements OnGatewayConnection, OnGatewayDisconnect
{
  constructor(
    @Inject(forwardRef(() => WebsocketService))
    private readonly websocketService: WebsocketService,
  ) {}

  @WebSocketServer() public server: Server;

  @SubscribeMessage('/battery')
  baterry(@MessageBody() data: string) {
    this.websocketService.newBatteryReading(Object(JSON.parse(data)).data);
  }

  @SubscribeMessage('/task_feedback')
  taskFeedback(@MessageBody() data: any) {

    const parsedData = Object(JSON.parse(data))

    console.log(`[websocketsGateway]    Received taskfeedback: ID ${parsedData._id}, type ${parsedData._type}`);

    if (!parsedData._id) throw new Error('Invalid data');

    switch (parsedData._type) {
      case 'GRAB':
        this.websocketService.handleGrabTask(parsedData._id);
        break;
      case 'DROP':
        this.websocketService.handleDropTask(parsedData._id);
        break;
      default:
        throw new Error('Invalid data');
    }
  }

  @SubscribeMessage('enqueue')
  enq(@MessageBody() data: any) {
    console.log('received enqueuee');
    console.log(data);
    this.websocketService.addPointToQueue(data);
  }

  @SubscribeMessage('emergency_stop')
  stop(@MessageBody() data: any) {
    console.log('received stop');
    console.log(data);
    this.websocketService.emergencyStop(data);
  }

  handleDisconnect(client: any) {
    console.log('[WebsocketGateway] client disconnected');
  }
  handleConnection(client: any, ...args: any[]) {
    console.log('[WebsocketGateway] client connected');
  }
}
