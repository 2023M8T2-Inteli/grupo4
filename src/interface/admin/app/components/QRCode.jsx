import React, { useState, useEffect } from "react";
import { QRCodeSVG } from "qrcode.react";
import QRCodeInstructions from "./QRCodeInstructions";

const QRCode = () => {
  const [qrcodeData, setQRCodeData] = useState({});

  const fetchQRCode = async () => {
    try {
      const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + "/qrcode");
      const data = await response.json();
      console.log("console.log: ", data);
      setQRCodeData(data);
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
    <div className="border-lg shadow-md w-full border-gray-100 border-[2px] rounded-md p-4 text-sm h-full my-auto">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">
          QR CODE - Vincular dispositivo
        </h1>
      </span>
      <div className="flex items-center mt-32 gap-9 justify-center">
        {qrcodeData.qr ? (
          <QRCodeSVG height={240} width={240} value={qrcodeData.qr} />
        ) : qrcodeData.isAuthenticated ? (
          <div className="text-center max-w-2xl">
            <h1 className="text-xl font-semibold mb-4">
              Dispositivo já está vinculado
            </h1>
            <p className="mb-4">
              Seu dispositivo já está autenticado e vinculado. Você pode agora
              fechar esta janela e usar o chatbot no seu WhatsApp.
            </p>
          </div>
        ) : (
          <div className="text-center">
            <h1 className="text-xl font-semibold mb-4">
              Aguardando o QR code ...
            </h1>
            <p>Por favor, aguarde. O QR code está sendo gerado.</p>
          </div>
        )}
        {qrcodeData.qr && <QRCodeInstructions />}
      </div>
    </div>
  );
};

export default QRCode;
