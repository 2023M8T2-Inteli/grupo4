import { Controller } from '@nestjs/common';
import { HandlerService } from './handler.service';

@Controller()
export class HandlerController {
  constructor(private readonly handlerService: HandlerService) {}
}
