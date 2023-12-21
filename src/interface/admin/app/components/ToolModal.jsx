import React, { useState } from "react";

const ToolModal = ({ isOpen, onClose, onSubmit }) => {
  const [formData, setFormData] = useState({
    name: "",
    price: "",
    minQuantity: "",
    maxQuantity: "",
    tag: "",
    pointX: "",
    pointY: "",
  });

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prevData) => ({ ...prevData, [name]: value }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    onSubmit(formData);
    setFormData({
      name: "",
      price: "",
      minQuantity: "",
      maxQuantity: "",
      tag: "",
      pointX: "",
      pointY: "",
    });
  };

  return (
    <div
      className={`fixed top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 bg-white p-4 rounded-md shadow-md ${
        isOpen ? "block" : "hidden"
      }`}
    >
      <h2 className="text-xl font-semibold mb-4">Adicionar Novo Item</h2>
      <form onSubmit={handleSubmit}>
        <label className="block mb-2">
          Nome:
          <input
            type="text"
            name="name"
            value={formData.name}
            onChange={handleChange}
            required
            className="border p-2 w-full"
          />
        </label>
        <label className="block mb-2">
          Preço:
          <input
            type="text"
            name="price"
            value={formData.price}
            onChange={handleChange}
            required
            className="border p-2 w-full"
          />
        </label>
        <label className="block mb-2">
          Quantidade mínima:
          <input
            type="text"
            name="minQuantity"
            value={formData.minQuantity}
            onChange={handleChange}
            required
            className="border p-2 w-full"
          />
        </label>
        <label className="block mb-2">
          Quantidade máxima:
          <input
            type="text"
            name="maxQuantity"
            value={formData.maxQuantity}
            onChange={handleChange}
            required
            className="border p-2 w-full"
          />
        </label>
        <label className="block mb-2">
            Tag:
            <input
              type="text"
              name="tag"
              value={formData.tag}
              onChange={handleChange}
              required
              className="border p-2 w-full"
            />
            </label>

            <label className="block mb-2">
            PointX:
            <input
              type="number"
              name="pointX"
              value={formData.pointX}
              onChange={handleChange}
              required
              className="border p-2 w-full"
            />
            </label>
            <label className="block mb-2">
            PointY:
            <input
              type="number"
              name="pointY"
              value={formData.pointY}
              onChange={handleChange}
              required
              className="border p-2 w-full"
            />
            </label>
        <button
          type="submit"
          className="bg-green-400 text-white py-2 px-4 rounded-md mr-2"
        >
          Adicionar
        </button>
        <button
          type="button"
          onClick={onClose}
          className="bg-gray-400 text-white py-2 px-4 rounded-md"
        >
          Fechar
        </button>
      </form>
    </div>
  );
};

export default ToolModal;
