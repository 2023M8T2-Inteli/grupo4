import React, { useState, useEffect } from "react";


const QRCode = () => {
  const [qrcode, setQRCode] = useState([]);

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchQRCode = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("http://52.5.70.100:3000/qrcode");
      const data = await response.json();
      console.log(data)
      setQRCode(data)
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };
  useEffect(() => {
    // Fetch history data when the component mounts
    fetchQRCode();

    // Fetch data every 10 seconds
    const intervalId = setInterval(fetchQRCode, 10000);

    // Clear the interval when the component is unmounted
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="border-lg shadow-md w-full border-gray-100 border-[2px] rounded-md p-4 text-sm">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">QR CODE</h1>
      </span>
      <div>
      </div>
    </div>
  );
};

export default QRCode;
