'use client'
import { useEffect, useState } from "react";
import Sidebar from "./components/Sidebar";
import Now from "./components/Now";
import Fila from "./components/Fila";
import History from "./components/History";

const page = () => {
  const [queue, setQueue] = useState([]);
  const [now, setNow] = useState([]);

  useEffect(() => {
    fetchOrders();

    const intervalId = setInterval(fetchOrders, 10000);

    // Clear the interval when the component is unmounted
    return () => clearInterval(intervalId);
  }, [])

  const fetchOrders = async () => {
    console.log(process.env.NEXT_PUBLIC_HOST + "/orders/queue")
    const response = await fetch(process.env.NEXT_PUBLIC_HOST + "/orders/queue");

    if (!response.ok) {
      throw new Error(`HTTP error! Status: ${response.status}`);
    }

    const data = await response.json();
    console.log(data)
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
        <div className="overflow-y-auto">
        <History/>
        </div>
        
      </div>
    </div>
  );
};

export default page;
