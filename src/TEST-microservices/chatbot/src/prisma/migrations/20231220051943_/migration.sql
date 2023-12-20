/*
  Warnings:

  - A unique constraint covering the columns `[messageId]` on the table `Transcription` will be added. If there are existing duplicate values, this will fail.

*/
-- CreateIndex
CREATE UNIQUE INDEX "Transcription_messageId_key" ON "Transcription"("messageId");
