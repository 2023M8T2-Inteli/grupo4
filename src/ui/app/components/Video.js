// VideoComponent.js
import React from "react";

export default function VideoComponent() {
  return (
    <video
      className="object-cover w-full h-full absolute inset-0"
      autoPlay
      muted
      loop
    >
      <source src="a.mp4" type="video/mp4" />
      {/* Add additional source elements for different video formats if needed */}
      Your browser does not support the video tag.
    </video>
  );
}
