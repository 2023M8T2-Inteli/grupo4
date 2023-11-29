import { NextResponse } from "next/server";


export async function GET(req) {
    NextResponse.setHeader
    res.setHeader('Content-Type', 'text/event-stream');
  res.setHeader('Cache-Control', 'no-cache');
  res.setHeader('Connection', 'keep-alive');
  res.flushHeaders();
    return NextResponse.json({ message: "Hello World" });
  }

