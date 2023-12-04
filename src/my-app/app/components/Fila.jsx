"use client";
import { useState } from "react";

const Fila = () => {
  const [queueData, setQueueData] = useState([
    {
      id: 1,
      location: "Warehouse A",
      estimatedArrival: "2023-12-05T10:30:00",
      requesterImage: "https://example.com/user1.jpg",
      requesterName: "John Doe",
    },
    {
      id: 2,
      location: "Office B",
      estimatedArrival: "2023-12-05T11:15:00",
      requesterImage: "https://example.com/user2.jpg",
      requesterName: "Jane Smith",
    },
    {
      id: 3,
      location: "Lab C",
      estimatedArrival: "2023-12-05T12:00:00",
      requesterImage: "https://example.com/user3.jpg",
      requesterName: "Bob Johnson",
    },
  ]);

  const fetchQueueData = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("YOUR_API_ENDPOINT");
      const data = await response.json();
      setQueueData(data);
    } catch (error) {
      console.error("Error fetching queue data:", error);
    }
  };

  return (
    <div className="border-lg shadow-md w-full h-full overflow-y-auto border-gray-100 border-[2px] rounded-md p-4 text-sm">
      <h1 className="text-2xl font-semibold mb-4">Na fila</h1>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Tool</th>
            <th className="py-2 px-4 border-b">Location</th>
            <th className="py-2 px-4 border-b">ETA</th>
            <th className="py-2 px-4 border-b">Requester</th>
          </tr>
        </thead>
        <tbody>
          {queueData.map((item) => (
            <tr key={item.id} className="border-b">
              <td className="py-2 px-4">{item.location}</td>
              <td className="py-2 px-4">{item.estimatedArrival}</td>
              <td className="py-2 px-4">
                <img
                  src={item.requesterImage}
                  alt={item.requesterName}
                  className="w-8 h-8 rounded-full mr-2"
                />
                <span>{item.requesterName}</span>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default Fila;
