'use client'

import Image from "next/image";
import MenuButton from "./components/MenuButton";
import { useState } from "react";
import {FaWhatsapp} from 'react-icons/fa'

export default function Home() {
  const [isPopupOpen, setPopupOpen] = useState(false);

  const togglePopup = () => {
    setPopupOpen(!isPopupOpen);
  };

  return (
    <div className="h-screen w-full flex justify-center items-center">
      <video
        className="object-cover w-full h-full absolute inset-0"
        autoPlay
        loop
        muted
      >
        <source src="expressions.mp4" type="video/mp4" />
        {/* Add additional source elements for different video formats if needed */}
        Your browser does not support the video tag.
      </video>
      <span className="absolute top-2 right-2 z-10">
        <div
          className="rounded-full border-2 w-14 h-14 border-lime-700 text-white flex items-center justify-center text-2xl"
          onClick={togglePopup}
        >
          <FaWhatsapp />
        </div>
      </span>
      {isPopupOpen && (
        <div className="fixed inset-0 flex items-center justify-center bg-black bg-opacity-50">
          <div className="bg-green-300 bg-opacity-30 p-6 rounded-lg backdrop-blur-md">
            {/* Popup Content */}
            <p className="text-white">Popup Content Goes Here</p>
            <button
              className="mt-4 bg-green-500 text-white py-2 px-4 rounded-full hover:bg-green-600 focus:outline-none focus:ring focus:border-green-300"
              onClick={togglePopup}
            >
              Close Popup
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
