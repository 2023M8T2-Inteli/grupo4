import React, { useState, useEffect } from "react";
import formatTime from "../utils/formatTime";
import DownloadButton from "./DownloadButton";

const History = () => {
  const [historyData, setHistoryData] = useState([]);

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchHistoryData = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch(
        process.env.NEXT_PUBLIC_BACKEND + "/orders/history"
      );
      const data = await response.json();
      setHistoryData(data);
      
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  useEffect(() => {
    // Fetch history data when the component mounts
    fetchHistoryData();

    // Fetch data every 10 seconds
    const intervalId = setInterval(fetchHistoryData, 10000);

    // Clear the interval when the component is unmounted
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="border-lg shadow-md w-full border-gray-100 border-[2px] rounded-md p-4 text-sm">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Histórico</h1>
        <DownloadButton data={historyData} filename={"OrderHistory"} />
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Item</th>
            <th className="py-2 px-4 border-b">Requisitante</th>
            <th className="py-2 px-4 border-b">Destino</th>
            <th className="py-2 px-4 border-b">Horário</th>
          </tr>
        </thead>
        <tbody className="">
          {historyData.map((item) => {
            
            return (
              <tr key={item?.id} className="border-b text-center">
                <td className="py-2 px-4">{item?.tool?.name}</td>
                <td className="py-2 px-4">{item?.user?.name}</td>
                <td className="py-2 px-4">{item?.point?.name}</td>
                <td className="py-2 px-4">{formatTime(item?.createdAt)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
};

export default History;
