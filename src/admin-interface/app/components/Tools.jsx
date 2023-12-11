"use client";
import React, { useState, useEffect } from "react";

const Tools = () => {
  const [tools, setTools] = useState([]);

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchTools = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("http://localhost:5000/tools");
      const data = await response.json();
      setTools(data);
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  useEffect(() => {
    // Fetch history data when the component mounts
    fetchTools();
    const intervalId = setInterval(fetchTools, 10000);

    // Clear the interval when the component is unmounted
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="border-lg h-full overflow-y-auto p-4 shadow-md border-gray-100 border-[2px] rounded-md mx-4">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Itens</h1>
        <button className=" border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2">
          BAIXAR
        </button>
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Item</th>
            <th className="py-2 px-4 border-b">Preço</th>
            <th className="py-2 px-4 border-b">Quantidade mínima</th>
            <th className="py-2 px-4 border-b">Quantidade máxima</th>
          </tr>
        </thead>
        <tbody>
          {tools.map((item) => (
            <tr key={item.id} className="border-b text-center">
              <td className="py-2 px-4">{item.name}</td>
              <td className="py-2 px-4">{item.price}</td>
              <td className="py-2 px-4">{item.minQuantity}</td>
              <td className="py-2 px-4">{item.maxQuantity}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default Tools;
