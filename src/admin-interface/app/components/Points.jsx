"use client";
import React, { useState, useEffect } from "react";

const Points = () => {
  const [points, setPoints] = useState([]);

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchPoints = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("http://localhost:5000/points");
      const data = await response.json();
      setPoints(data);
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  useEffect(() => {
    // Fetch history data when the component mounts
    fetchPoints();
    const intervalId = setInterval(fetchPoints, 10000);

    // Clear the interval when the component is unmounted
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="border-lg h-full overflow-y-auto p-4 shadow-md border-gray-100 border-[2px] rounded-md mx-4">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Destinos</h1>
        <button className=" border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2">
          BAIXAR
        </button>
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Local</th>
            <th className="py-2 px-4 border-b">Coordenada X</th>
            <th className="py-2 px-4 border-b">Coordenada Y</th>
            <th className="py-2 px-4 border-b">Coordenada Z</th>
          </tr>
        </thead>
        <tbody>
          {points.map((item) => (
            <tr key={item.id} className="border-b text-center">
              <td className="py-2 px-4">{item.name}</td>
              <td className="py-2 px-4">{item.pointX}</td>
              <td className="py-2 px-4">{item.pointY}</td>
              <td className="py-2 px-4">{item.pointZ}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default Points;
