'use client'
import { useState, useEffect } from "react";

export default function VideoComponent({ setEmotion, emotion }) {


  useEffect(() => {
    console.log("Emotion prop changed:", emotion);
   
  }, [emotion]);
  return (
    <div>
      <video className="object-cover w-full h-full absolute inset-0" autoPlay muted loop key={emotion} >
       <source src={`https://d17sdup6iumur7.cloudfront.net/${emotion}.mp4`} type="video/mp4" />
        Your browser does not support the video tag.
      </video>
    </div>
  );
}
