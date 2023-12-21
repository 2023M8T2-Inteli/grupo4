import React, { useState, useEffect } from "react";

const QRCodeInstructions = () => {
  return (
    <div className="mt-4">
      <h1 className="text-2xl font-bold mb-4">Use WhatsApp Web to Chatbot</h1>
      <ol className="list-decimal list-inside text-left">
        <li>Abrir o WhatsApp no seu telefone.</li>
        <li>
          Toque no Menu ou Configurações e selecione Dispositivos Linkados.
        </li>
        <li>Toque em Vincular um dispositivo.</li>
        <li>Aponte seu telefone para esta tela para capturar o código QR.</li>
      </ol>
    </div>
  );
};

export default QRCodeInstructions;
