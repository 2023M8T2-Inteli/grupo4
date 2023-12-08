'use client'
import { useState } from "react";
import VideoComponent from "./components/Video";
import Talk from "./components/Talk";

export default function Home() {

  return (
    <div className="h-screen w-full flex justify-center items-center bg-cover">
      <VideoComponent />
      <Talk />
    </div>
  );
}
