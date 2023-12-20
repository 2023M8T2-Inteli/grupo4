import React, { useState, useEffect } from "react";
// import QRCode from "react-qr-code";
import {QRCodeSVG} from 'qrcode.react';


const QRCodeComponent = () => {
  const [qrcodeData, setQRCodeData] = useState({});

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchQRCode = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("http://10.128.83.103:3000/qrcode")
      const data = await response.json();
      console.log("console.log: ", data)
      setQRCodeData(data)
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  useEffect(() => {
    // Fetch history data when the component mounts
    fetchQRCode();

    // Fetch data every 10 seconds
    const intervalId = setInterval(fetchQRCode, 1000);

    // Clear the interval when the component is unmounted
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="border-lg shadow-md w-full border-gray-100 border-[2px] rounded-md p-4 text-sm h-full">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">QR CODE</h1>
      </span>
      <div className="flex items-center justify-center">
        {qrcodeData.qr 
        ?
        <QRCodeSVG value={qrcodeData.qr} /> 
        :
        qrcodeData.isAuthenticated 
        ?
        <p>Já autenticado.</p>
        : 
        <p>Ainda não foi gerado.</p>}
      </div>
    </div>  
  );
};

export default QRCodeComponent;
