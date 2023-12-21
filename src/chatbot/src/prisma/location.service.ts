import { Inject, Injectable } from '@nestjs/common';
import { Point as PrismaPoint } from '@prisma/client';
import { PrismaService } from './prisma.service';

export class LocationDoesntExists extends Error {
  constructor(message: string = 'Tool doesnt exists') {
    super(message);
    this.name = 'ToolDoesntExists';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, LocationDoesntExists);
    }
  }
}

export class TablePointsIsEmpty extends Error {
  constructor(message: string = 'Table tool is empty') {
    super(message);
    this.name = 'TableToolIsEmpty';
    // Mantém o stack trace em V8
    if (Error.captureStackTrace) {
      Error.captureStackTrace(this, TablePointsIsEmpty);
    }
  }
}

@Injectable()
export class LocationService {
  constructor(@Inject(PrismaService) private prisma: PrismaService) {}

  async createLocation(
    name: string,
    pointX: number,
    pointY: number,
  ): Promise<PrismaPoint> {
    const location = await this.prisma.point.create({
      data: {
        name,
        pointX,
        pointY,
        pointZ: 0.0,
      },
    });
    return location;
  }

  async getAllLocations(): Promise<PrismaPoint[]> {
    const locations = await this.prisma.point.findMany();

    if (!locations) throw new TablePointsIsEmpty();

    return locations;
  }

  async getLocationByCoords(coords: number[]): Promise<PrismaPoint> {
    const location = await this.prisma.point.findFirst({
      where: {
        pointX: {
          equals: coords[0],
        },
        pointY: {
          equals: coords[1],
        },
      },
    });

    if (!location) throw new LocationDoesntExists();

    return location;
  }

  async locationExists(coords: number[]): Promise<boolean> {
    try {
      await this.getLocationByCoords(coords);
      return true;
    } catch {
      return false;
    }
  }

  async getLocationIdByCoords(coords: number[]): Promise<string> {
    const location = await this.getLocationByCoords(coords);
    return location.id;
  }

  async getLocationById(id: string): Promise<PrismaPoint> {
    const location = await this.prisma.point.findFirst({
      where: {
        id: id,
      },
    });

    if (!location) throw new LocationDoesntExists();

    return location;
  }

  async deleteLocation(id: string): Promise<PrismaPoint> {
    const location = await this.prisma.point.delete({
      where: {
        id: id,
      },
    });

    if (!location) throw new LocationDoesntExists();

    return location;
  }
  async updateLocation(id: string, name: string, pointX: number, pointY: number): Promise<PrismaPoint> {
    const data: Record<string, any> = {};

    // Only include properties in the data object if they are provided
    if (name !== undefined) data.name = name;
    if (pointX !== undefined) data.pointX = pointX;
    if (pointY !== undefined) data.pointY = pointY;

    const location = await this.prisma.point.update({
        where: {
            id: id,
        },
        data: data,
    });

    if (!location) throw new LocationDoesntExists();

    return location;
}
}
