"use client";
import React, { useState, useEffect } from "react";
import DownloadButton from "./DownloadButton";

import { FaTrash } from "react-icons/fa";
import ToolModal from "./ToolModal";

const Tools = () => {
  const [tools, setTools] = useState([]);
  const [editingTool, setEditingTool] = useState(null); // To track the tool being edited
  const [editedData, setEditedData] = useState({}); // To store edited data before sending to the server
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleOpenModal = () => {
    setIsModalOpen(true);
  };

  const handleCloseModal = () => {
    setIsModalOpen(false);
  };

  const handleAddTool = async (formData) => {
    try {
      const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + '/tools', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      if (response.ok) {
        console.log('Tool added successfully:', formData);
        fetchTools();
      } else {
        console.error('Failed to add tool:', response.status, response.statusText);
      }
    } catch (error) {
      console.error('Error adding tool:', error);
    }

    handleCloseModal();
  };

  // Function to fetch tools data (replace with your actual data fetching logic)
  const fetchTools = async () => {
    try {
      // Replace this with your API endpoint or data source
      const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + "/tools");
      const data = await response.json();
      setTools(data);
    } catch (error) {
      console.error("Error fetching tools data:", error);
    }
  };

  const handleDelete = async (toolId) => {
    try {
      const response = await fetch(`${process.env.NEXT_PUBLIC_BACKEND}/tools/${toolId}`, {
        method: "DELETE",
      });

      if (response.ok) {
        setTools((prevTools) => prevTools.filter((tool) => tool.id !== toolId));
      } else {
        console.error("Failed to delete tool on the server");
      }
    } catch (error) {
      console.error("Error deleting tool:", error);
    }
  };


  const handleDoubleClick = (tool) => {
    setEditingTool(tool);
    setEditedData({
      name: tool.name,
      price: tool.price,
      minQuantity: tool.minQuantity,
      maxQuantity: tool.maxQuantity,
    });
  };

  const handleBlur = async () => {
    // Send the updated data to the server on blur
    console.log(editedData)
    try {
      const response = await fetch(`${process.env.NEXT_PUBLIC_BACKEND}/tools/${editingTool.id}`, {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(editedData),
      });

      if (response.ok) {
        // Update the local state with the edited data
        setTools((prevTools) =>
          prevTools.map((tool) =>
            tool.id === editingTool.id ? { ...tool, ...editedData } : tool
          )
        );
      } else {
        console.error("Failed to update tool on the server");
      }
    } catch (error) {
      console.error("Error updating tool:", error);
    }

    // Clear the editing state
    setEditingTool(null);
    setEditedData({});
  };

  useEffect(() => {
    // Fetch tools data when the component mounts
    fetchTools();
    const intervalId = setInterval(fetchTools, 10000);

    // Clear the interval when the component is unmounted
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="border-lg h-full overflow-y-auto p-4 shadow-md border-gray-100 border-[2px] rounded-md mx-4">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Itens</h1>
        <DownloadButton data={tools} filename={'Tools'}/>
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Item</th>
            <th className="py-2 px-4 border-b">Preço</th>
            <th className="py-2 px-4 border-b">Quantidade mínima</th>
            <th className="py-2 px-4 border-b">Quantidade máxima</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          {tools.map((tool) => (
            <tr
              key={tool.id}
              className={`border-b text-center `}
              onDoubleClick={() => handleDoubleClick(tool)}
            >
              <td className="py-2 px-4">
                {editingTool && editingTool.id === tool.id ? (
                  <input
                    type="text"
                    value={editedData.name}
                    onChange={(e) => setEditedData({ ...editedData, name: e.target.value })}
                    onBlur={handleBlur}
                  />
                ) : (
                  tool.name
                )}
              </td>
              <td className="py-2 px-4">
                {editingTool && editingTool.id === tool.id ? (
                  <input
                    type="text"
                    value={editedData.price}
                    onBlur={handleBlur}

                    onChange={(e) => setEditedData({ ...editedData, price: e.target.value })}
                  />
                ) : (
                  tool.price
                )}
              </td>
              <td className="py-2 px-4">
                {editingTool && editingTool.id === tool.id ? (
                  <input
                    type="text"
                    value={editedData.minQuantity}
                    onBlur={handleBlur}

                    onChange={(e) =>
                      setEditedData({ ...editedData, minQuantity: e.target.value })
                    }
                  />
                ) : (
                  tool.minQuantity
                )}
              </td>
              <td className="py-2 px-4">
                {editingTool && editingTool.id === tool.id ? (
                  <input
                    type="text"
                    value={editedData.maxQuantity}
                    onBlur={handleBlur}

                    onChange={(e) =>
                      setEditedData({ ...editedData, maxQuantity: e.target.value })
                    }
                  />
                ) : (
                  tool.maxQuantity
                )}
              </td>
              <td className="py-2 px-4">
                <button onClick={() => handleDelete(tool.id)}>
                  <FaTrash />
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      <ToolModal isOpen={isModalOpen} onClose={handleCloseModal} onSubmit={handleAddTool} />
      <button
        className="border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2 mt-2"
        onClick={handleOpenModal}
      >Adicionar Novo</button>
    </div>
  );
};

export default Tools;
