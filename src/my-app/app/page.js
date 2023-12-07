'use client'

import React, { useEffect, useState } from "react";
import Sidebar from "./components/Sidebar";
import Now from "./components/Now";
import Fila from "./components/Fila";
import History from "./components/History";

const page = () => {
  const [queue, setQueue] = useState([]);
  const [now, setNow] = useState([]);

  useEffect(() => {
    fetchOrders();
  })

  const fetchOrders = async () => {
    const response = await fetch("http://localhost:5000/orders/queue");

    if (!response.ok) {
      throw new Error(`HTTP error! Status: ${response.status}`);
    }

    const data = await response.json();
    setNow(data[0])
    setQueue(data.slice(1));
  };

  return (
    <div className="h-screen w-full flex">
      <Sidebar />
      <div className="flex flex-col w-full">
        <div className="flex gap-4 p-4 justify-between w-full h-1/2">
          <Now now={now}/>
          <Fila queue={queue}/>
        </div>
        <div className="">
        <History/>
        </div>
        
      </div>
    </div>
  );
};

export default page;
