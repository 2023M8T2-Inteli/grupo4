"use client";
import React, { useState, useEffect } from "react";

const Users = () => {
  const [users, setUserData] = useState([
   
  ]);

  // Function to fetch history data (replace with your actual data fetching logic)
  const fetchUsers = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch("http://localhost:5000/users");
      const data = await response.json();
      setUserData(data);
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  useEffect(() => {
    // Fetch history data when the component mounts
    fetchUsers();
  }, []);

  return (
    <div className="border-lg h-full overflow-y-auto p-4 shadow-md border-gray-100 border-[2px] rounded-md mx-4">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Usuários</h1>
        <button className=" border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2">
          BAIXAR
        </button>
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Nome</th>
            <th className="py-2 px-4 border-b">Celular</th>
            <th className="py-2 px-4 border-b">Permissões</th>
          </tr>
        </thead>
        <tbody>
          {users.map((item) => (
            <tr key={item.id} className="border-b">
              <td className="py-2 px-4">{item.name}</td>
              <td className="py-2 px-4">{item.cellPhone}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default Users;
