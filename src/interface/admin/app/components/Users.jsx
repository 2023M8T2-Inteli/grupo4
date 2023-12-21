"use client";
import React, { useState, useEffect } from "react";
import DownloadButton from "./DownloadButton";
import UserModal from "./UserModal"; // Import the UserModal component
import { FaTrash } from "react-icons/fa";


const Users = () => {
  const [users, setUserData] = useState([]);
  const [editingId, setEditingId] = useState(null);
  const [isModalOpen, setIsModalOpen] = useState(false);


  const fetchUsers = async () => {
    try {
      const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + "/users");
      const data = await response.json();
      setUserData(data);
    } catch (error) {
      console.error("Error fetching history data:", error);
    }
  };

  const handleOpenModal = () => {
    setIsModalOpen(true);
  };

  const handleCloseModal = () => {
    setIsModalOpen(false);
  };

  const handleDelete = async (id) => {
    try {
      await fetch(`${process.env.NEXT_PUBLIC_BACKEND}/users/${id}`, {
        method: "DELETE",
      });
      fetchUsers();
    } catch (error) {
      console.error(`Error deleting user ${id}:`, error);
    }
  };

  const handleDoubleClick = (id) => {
    setEditingId(id);
  };

  const handleBlur = async (id, field, value) => {
    try {
      // Update the user data on the server using a PUT request
      await fetch(`${process.env.NEXT_PUBLIC_BACKEND}/users/${id}`, {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ [field]: value }),
      });
      // Fetch updated user data after the edit
      fetchUsers();
    } catch (error) {
      console.error(`Error updating ${field} for user ${id}:`, error);
    } finally {
      setEditingId(null);
    }
  };

  const handleAddUser = async (formData) => {
    try {
      const response = await fetch(process.env.NEXT_PUBLIC_BACKEND + "/users", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(formData),
      });

      if (response.ok) {
        fetchUsers();
      } else {
        console.error("Failed to add user:", response.status, response.statusText);
      }
    } catch (error) {
      console.error("Error adding user:", error);
    }

    handleCloseModal();
  };

  const handleRoleChange = async (id, value) => {
    try {
      // Update the role for the user on the server using a PUT request
      await fetch(`${process.env.NEXT_PUBLIC_BACKEND}/users/${id}`, {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ role: value }),
      });
      // Fetch updated user data after the edit
      fetchUsers();
    } catch (error) {
      console.error(`Error updating role for user ${id}:`, error);
    } finally {
      setEditingId(null);
    }
  };

  useEffect(() => {
    fetchUsers();
  }, []);

  return (
    <div className="border-lg h-full overflow-y-auto p-4 shadow-md border-gray-100 border-[2px] rounded-md mx-4">
      <span className="flex justify-between m-2">
        <h1 className="text-2xl font-semibold mb-4">Usuários</h1>
        <DownloadButton data={users} filename={'Users'}/>
      </span>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100">
            <th className="py-2 px-4 border-b">Nome</th>
            <th className="py-2 px-4 border-b">Celular</th>
            <th className="py-2 px-4 border-b">Permissões</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          {users.map((item) => (
            <tr key={item.id} className="border-b text-center">
              <td
                className="py-2 px-4 cursor-pointer"
                onDoubleClick={() => handleDoubleClick(item.id)}
              >
                {editingId === item.id ? (
                  <input
                    type="text"
                    defaultValue={item.name}
                    onBlur={(e) =>
                      handleBlur(item.id, "name", e.target.value)
                    }
                  />
                ) : (
                  item.name
                )}
              </td>
              <td
                className="py-2 px-4 cursor-pointer"
                onDoubleClick={() => handleDoubleClick(item.id)}
              >
                {editingId === item.id ? (
                  <input
                    type="text"
                    defaultValue={item.cellPhone}
                    onBlur={(e) =>
                      handleBlur(item.id, "cellPhone", e.target.value)
                    }
                  />
                ) : (
                  item.cellPhone
                )}
              </td>
              <td
                className="py-2 px-4 cursor-pointer"
                onDoubleClick={() => handleDoubleClick(item.id)}
              >
                {editingId === item.id ? (
                  <select
                    defaultValue={item.role}
                    onBlur={(e) => handleRoleChange(item.id, e.target.value)}
                  >
                    <option value="ADMIN">ADMIN</option>
                    <option value="USER">USER</option>
                    <option value="LEAD">LEAD</option>
                  </select>
                ) : (
                  item.role
                )}
              </td>
              <td>
                <button
                  className="px-2 py-1 rounded-md mr-2"
                  onClick={() => handleDelete(item.id)}
                >
                  <FaTrash />
                </button>
              </td>

            </tr>
          ))}
        </tbody>
      </table>

      <UserModal
        isOpen={isModalOpen}
        onClose={handleCloseModal}
        onSubmit={handleAddUser}
      />
    </div>
  );
};

export default Users;
