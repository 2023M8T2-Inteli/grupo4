'use client'

import { useState, useEffect } from "react";
import {FaWhatsapp} from 'react-icons/fa'


export default function Home() {

  const [isPopupOpen, setPopupOpen] = useState(false);
  const [qrCode, setQrCode] = useState(null);

  const togglePopup = () => {
    setPopupOpen(!isPopupOpen);
  };

  useEffect(() => {
    if (isPopupOpen) {
      fetch('http://localhost:5000/')
      .then((res) => {
        if (!res.ok) {
          throw new Error(`Network response was not ok: ${res.statusText}`);
        }
        return res.text()
      })
      .then((data) => {
        setQrCode(data);
      })
      .catch((error) => {
        console.error('Error fetching data:', error);
      });
    } else {
      
    }
  }, [isPopupOpen])

  return (
    <div className="h-screen w-full flex justify-center items-center">
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
            <div dangerouslySetInnerHTML={{ __html: qrCode }} />
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
