/*
  Warnings:

  - Added the required column `pointX` to the `Tool` table without a default value. This is not possible if the table is not empty.
  - Added the required column `pointY` to the `Tool` table without a default value. This is not possible if the table is not empty.
  - Added the required column `pointZ` to the `Tool` table without a default value. This is not possible if the table is not empty.

*/
-- AlterTable
ALTER TABLE "Tool" ADD COLUMN     "pointX" DOUBLE PRECISION NOT NULL,
ADD COLUMN     "pointY" DOUBLE PRECISION NOT NULL,
ADD COLUMN     "pointZ" DOUBLE PRECISION NOT NULL;
