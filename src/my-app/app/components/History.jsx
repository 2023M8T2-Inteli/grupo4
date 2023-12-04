"use client";
import React, { useState, useEffect } from "react";

const History = () => {
  const [historyData, setHistoryData] = useState([
    {
      id: 1,
      ferramenta: "Drill",
      requisitante: "Alice Johnson",
      destino: "Workshop A",
      status: "Delivered",
      horario: "2023-12-05T09:45:00",
    },
    {
      id: 2,
      ferramenta: "Wrench Set",
      requisitante: "Bob Smith",
      destino: "Maintenance Room",
      status: "In Progress",
      horario: "2023-12-05T10:30:00",
    },
    {
      id: 3,
      ferramenta: "Screwdriver",
      requisitante: "Charlie Brown",
      destino: "Assembly Line",
      status: "Completed",
      horario: "2023-12-05T11:15:00",
    },
  ]);

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchHistoryData = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("YOUR_API_ENDPOINT_FOR_HISTORY");
      const data = await response.json();
      setHistoryData(data);
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  useEffect(() => {
    // Fetch history data when the component mounts
    fetchHistoryData();
  }, []);

  return (
    <div className="border-lg h-full overflow-y-auto p-4 shadow-md border-gray-100 border-[2px] rounded-md m-4">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Histórico</h1>
        <button className=" border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2">
          BAIXAR
        </button>
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Ferramenta</th>
            <th className="py-2 px-4 border-b">Requisitante</th>
            <th className="py-2 px-4 border-b">Destino</th>
            <th className="py-2 px-4 border-b">Status</th>
            <th className="py-2 px-4 border-b">Horário</th>
          </tr>
        </thead>
        <tbody>
          {historyData.map((item) => (
            <tr key={item.id} className="border-b">
              <td className="py-2 px-4">{item.ferramenta}</td>
              <td className="py-2 px-4">{item.requisitante}</td>
              <td className="py-2 px-4">{item.destino}</td>
              <td className="py-2 px-4">{item.status}</td>
              <td className="py-2 px-4">{item.horario}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default History;
