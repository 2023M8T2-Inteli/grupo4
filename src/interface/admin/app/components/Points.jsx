"use client";
import React, { useState, useEffect } from "react";
import DownloadButton from "./DownloadButton";
import PointModal from "./PointModal";

import { FaTrash } from "react-icons/fa";
const Points = () => {
  const [points, setPoints] = useState([]);
  const [editingPoint, setEditingPoint] = useState(null);
  const [editedData, setEditedData] = useState({});
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleOpenModal = () => {
    setIsModalOpen(true);
  };

  const handleCloseModal = () => {
    setIsModalOpen(false);
  };

  const handleAddPoint = async (formData) => {
    try {
      const response = await fetch(
        process.env.NEXT_PUBLIC_BACKEND + "/points",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(formData),
        }
      );

      if (response.ok) {
        // Optionally, you can handle the successful response
        console.log("Point added successfully:", formData);
        fetchPoints();
      } else {
        // Handle the case where the server returned an error
        console.error(
          "Failed to add point:",
          response.status,
          response.statusText
        );
      }
    } catch (error) {
      // Handle network or other errors
      console.error("Error adding point:", error);
    }

    // Close the modal after handling the API call
    handleCloseModal();
  };

  const fetchPoints = async () => {
    try {
      const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + "/points");
      const data = await response.json();
      setPoints(data);
    } catch (error) {
      console.error("Error fetching points data:", error);
    }
  };

  const handleDoubleClick = (point) => {
    setEditingPoint(point);
    setEditedData({
      name: point.name,
      pointX: point.pointX,
      pointY: point.pointY,
      pointZ: point.pointZ,
    });
  };

  const handleBlur = async () => {
    try {
      const response = await fetch(
        process.env.NEXT_PUBLIC_BACKEND + `/points/${editingPoint.id}`,
        {
          method: "PUT",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(editedData),
        }
      );

      if (response.ok) {
        setPoints((prevPoints) =>
          prevPoints.map((p) =>
            p.id === editingPoint.id ? { ...p, ...editedData } : p
          )
        );
      } else {
        console.error("Failed to update point on the server");
      }
    } catch (error) {
      console.error("Error updating point:", error);
    }

    setEditingPoint(null);
    setEditedData({});
  };

  const handleDelete = async (pointId) => {
    try {
      const response = await fetch(
        `${process.env.NEXT_PUBLIC_BACKEND}/points/${pointId}`,
        {
          method: "DELETE",
        }
      );

      if (response.ok) {
        setPoints((prevPoints) => prevPoints.filter((p) => p.id !== pointId));
      } else {
        console.error("Failed to delete point on the server");
      }
    } catch (error) {
      console.error("Error deleting point:", error);
    }
  };

  useEffect(() => {
    fetchPoints();
    const intervalId = setInterval(fetchPoints, 10000);
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="border-lg h-full overflow-y-auto p-4 shadow-md border-gray-100 border-[2px] rounded-md mx-4">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Destinos</h1>
        <DownloadButton data={points} filename={"Points"} />
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Local</th>
            <th className="py-2 px-4 border-b">Coordenada X</th>
            <th className="py-2 px-4 border-b">Coordenada Y</th>
            <th className="py-2 px-4 border-b">Coordenada Z</th>
            <th className="py-2 px-4 border-b"></th>
          </tr>
        </thead>
        <tbody>
          {points.map((point) => (
            <tr
              key={point.id}
              className={`border-b text-center`}
              onDoubleClick={() => handleDoubleClick(point)}
              tabIndex="0"
            >
              <td className="py-2 px-4">
                {editingPoint && editingPoint.id === point.id ? (
                  <input
                    type="text"
                    value={editedData.name}
                    onChange={(e) =>
                      setEditedData({ ...editedData, name: e.target.value })
                    }
                    onBlur={handleBlur}
                    name="name"
                  />
                ) : (
                  point.name
                )}
              </td>
              <td className="py-2 px-4">
                {editingPoint && editingPoint.id === point.id ? (
                  <input
                    type="text"
                    value={editedData.pointX}
                    onChange={(e) =>
                      setEditedData({ ...editedData, pointX: e.target.value })
                    }
                    onBlur={handleBlur}
                    name="pointX"
                  />
                ) : (
                  point.pointX
                )}
              </td>
              <td className="py-2 px-4">
                {editingPoint && editingPoint.id === point.id ? (
                  <input
                    type="text"
                    value={editedData.pointY}
                    onChange={(e) =>
                      setEditedData({ ...editedData, pointY: e.target.value })
                    }
                    onBlur={handleBlur}
                    name="pointY"
                  />
                ) : (
                  point.pointY
                )}
              </td>
              <td className="py-2 px-4">
                {editingPoint && editingPoint.id === point.id ? (
                  <input
                    type="text"
                    value={editedData.pointZ}
                    onChange={(e) =>
                      setEditedData({ ...editedData, pointZ: e.target.value })
                    }
                    onBlur={handleBlur}
                    name="pointZ"
                  />
                ) : (
                  point.pointZ
                )}
              </td>
              <td className="py-2 px-4">
                <button onClick={() => handleDelete(point.id)}>
                  <FaTrash />
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>

      <button
        className="border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2 mt-2"
        onClick={handleOpenModal}
      >
        Adicionar Novo
      </button>
      <PointModal
        isOpen={isModalOpen}
        onClose={handleCloseModal}
        onSubmit={handleAddPoint}
      />
    </div>
  );
};

export default Points;
