import React, { useState, useEffect } from "react";
import {QRCodeSVG} from 'qrcode.react';


const QRCode = () => {
  const [qrcodeData, setQRCodeData] = useState({});


  const fetchQRCode = async () => {
    try {
      const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + "/qrcode");
      const data = await response.json();
      console.log("console.log: ", data)
      setQRCodeData(data)
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  useEffect(() => {
    fetchQRCode();
    const intervalId = setInterval(fetchQRCode, 1000);
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

export default QRCode;
