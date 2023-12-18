import React, { useState, useEffect } from "react";
import formatTime from "../utils/formatTime";

const History = () => {
  const [historyData, setHistoryData] = useState([]);

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchHistoryData = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("http://localhost:5000/orders/history");
      const data = await response.json();
      setHistoryData(data);
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };
  const downloadCSV = () => {
    const convertToCSV = (data) => {
      const rows = data.map((row) => Object.values(row).join(','));
      return ['sep=,', Object.keys(data[0]).join(','), ...rows].join('\n');
    };
    // TODO: Consertar o erro em que o data vem vazio, verificar se é um erro no endpoint ou na conexão com o banco
    
    // Convert the data to CSV format
    const csvData = convertToCSV(data);
  
    // Create a Blob containing the CSV data
    const blob = new Blob([csvData], { type: 'text/csv' });
  
    // Create a download link
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = 'data.csv';
  
    // Append the link to the DOM and trigger the download
    document.body.appendChild(link);
    link.click();
  
    // Remove the link from the DOM
    document.body.removeChild(link);
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
        <button onClick={downloadCSV} className="border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2">
          BAIXAR
        </button>
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
          {historyData.map((item) => (
            <tr key={item.id} className="border-b text-center">
              <td className="py-2 px-4">{item.tool.name}</td>
              <td className="py-2 px-4">{item.user.name}</td>
              <td className="py-2 px-4">{item.point.name}</td>
              <td className="py-2 px-4">{formatTime(item.createdAt)}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default History;
