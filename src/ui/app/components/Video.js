'use client'

import { useState, useEffect } from "react";
import fs from "fs";
import path from "path";

export default function VideoComponent() {

  return (
    <div>
      <video
        className="object-cover w-full h-full absolute inset-0"
        autoPlay
        muted
        loop
      >
        <source src="happy.mp4" type="video/mp4" />
        Your browser does not support the video tag.
      </video>
      
    </div>
  );
}
