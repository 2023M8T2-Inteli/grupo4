-- CreateTable
CREATE TABLE "GPTFunctionCalling" (
    "id" TEXT NOT NULL,
    "functionCallObject" TEXT NOT NULL,
    "functionName" TEXT NOT NULL,
    "functionResponse" TEXT,
    "messageId" TEXT NOT NULL,
    "userId" UUID NOT NULL,
    "createdAt" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,

    CONSTRAINT "GPTFunctionCalling_pkey" PRIMARY KEY ("id")
);

-- AddForeignKey
ALTER TABLE "GPTFunctionCalling" ADD CONSTRAINT "GPTFunctionCalling_userId_fkey" FOREIGN KEY ("userId") REFERENCES "User"("id") ON DELETE RESTRICT ON UPDATE CASCADE;
