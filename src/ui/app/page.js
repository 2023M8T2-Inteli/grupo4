// Home.js
'use client'
import { useState } from "react";
import Popup from "./components/Popup";
import WhatsappButton from "./components/WhatsappComponent";
import VideoComponent from "./components/Video";
import AudioPlayer from "./components/SSE";


export default function Home() {
  const [isPopupOpen, setPopupOpen] = useState(false);

  const togglePopup = () => {
    setPopupOpen(!isPopupOpen);
  };

  return (
    <div className="h-screen w-full flex justify-center items-center">
      <VideoComponent />
      <AudioPlayer />
      <WhatsappButton onClick={togglePopup} />
      <Popup isOpen={isPopupOpen} togglePopup={togglePopup} />
    </div>
  );
}
