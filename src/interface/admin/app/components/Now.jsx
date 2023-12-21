'use client'
import { useEffect, useState } from 'react';

const Now = ({now}) => {
  const [battery, setBattery] = useState(0);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + '/websockets/battery');
        const data = await response.json();
        console.log(data);
        setBattery(data);
      } catch (error) {
        console.error('Error fetching data:', error);
      }
    };

    fetchData(); // Initial fetch

    // Set up interval to fetch data every 10 seconds
    const intervalId = setInterval(fetchData, 10000);

    // Clean up the interval when the component unmounts
    return () => clearInterval(intervalId);
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
