// Popup.js
import { useState, useEffect } from "react";
import { FaWhatsapp } from "react-icons/fa";

export default function Popup({ isOpen, togglePopup }) {
  const [qrCode, setQrCode] = useState(null);

  useEffect(() => {
    if (isOpen) {
      fetch("http://localhost:5000/")
        .then((res) => {
          if (!res.ok) {
            throw new Error(
              `Network response was not ok: ${res.statusText}`
            );
          }
          return res.text();
        })
        .then((data) => {
          setQrCode(data);
        })
        .catch((error) => {
          console.error("Error fetching data:", error);
        });
    }
  }, [isOpen]);

  return (
    isOpen && (
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
    )
  );
}
