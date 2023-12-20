'use client'
import React, { useEffect } from 'react';
import {io} from 'socket.io-client';


const Now = ({now}) => {
  const [battery, setBattery] = React.useState(0);

  useEffect(() => {
    console.log('Connecting to server...')
    // Create a Socket.IO client and connect to the server
    const socket = io(process.env.NEXT_PUBLIC_SOCKET); // Replace with your server URL
    console.log(socket)
    // Handle events from the server
    socket.on('battery', (data) => {
      console.log('Received message from server:', data);
      let quantity = JSON.parse(data);
      console.log(quantity)
      setBattery(String(quantity.data).substring(0, 2));
    });
  }, []);

  return (
    <div className="shadow-md border-gray-100 border-[2px] rounded-md flex flex-col justify-between p-4 w-full h-full">
      <span className="flex items-center justify-between">
        <h1 className="text-2xl font-semibold">Agora</h1>
        <h3>Bateria: {battery}%</h3>
      </span>
      <span>
        <h3 className="text-gray-400 font-semibold text-sm">DESTINO</h3>
        <h1 className="text-3xl text-bold">{now?.point?.name}</h1>
      </span>

      <span className="flex justify-between">
        <h3>{now?.tool?.name}</h3>
        <h3>{now?.user?.name}</h3>
      </span>
      <button className="w-full border-red-400 border-[1px] text-red-400 rounded-md">
        PARAR
      </button>
    </div>
  );
};

export default Now;
