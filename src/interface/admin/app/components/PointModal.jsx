import React, { useState } from "react";

const PointModal = ({ isOpen, onClose, onSubmit }) => {
  const [name, setName] = useState("");
  const [pointX, setPointX] = useState("");
  const [pointY, setPointY] = useState("");
  const [pointZ, setPointZ] = useState("");

  const handleInputChange = (setter) => (e) => setter(e.target.value);

  const handleSubmit = () => {
    // Add validation or additional logic if needed
    onSubmit({ name, pointX, pointY, pointZ });
    onClose();
  };

  return (
    <div
      className={`fixed inset-0 z-50 overflow-auto bg-black bg-opacity-50 ${
        isOpen ? "" : "hidden"
      }`}
    >
      <div className="relative w-96 p-6 mx-auto my-8 bg-white rounded-md shadow-md">
        <h2 className="text-2xl font-semibold mb-4">Adicionar Novo Ponto</h2>
        <form>
          <div className="mb-4">
            <label className="block text-sm font-medium text-gray-700">
              Nome:
            </label>
            <input
              type="text"
              value={name}
              name="name"
              onChange={handleInputChange(setName)}
              className="w-full px-3 py-2 border rounded-md"
            />
          </div>
          <div className="mb-4">
            <label className="block text-sm font-medium text-gray-700">
              Coordenada X:
            </label>
            <input
              type="text"
              value={pointX}
              name="pointX"
              onChange={handleInputChange(setPointX)}
              className="w-full px-3 py-2 border rounded-md"
            />
          </div>
          <div className="mb-4">
            <label className="block text-sm font-medium text-gray-700">
              Coordenada Y:
            </label>
            <input
              type="text"
              value={pointY}
              name="pointY"
              onChange={handleInputChange(setPointY)}
              className="w-full px-3 py-2 border rounded-md"
            />
          </div>
          <div className="mb-4">
            <label className="block text-sm font-medium text-gray-700">
              Coordenada Z:
            </label>
            <input
              type="text"
              value={pointZ}
              name="pointZ"
              onChange={handleInputChange(setPointZ)}
              className="w-full px-3 py-2 border rounded-md"
            />
          </div>
          <button
            type="button"
            onClick={handleSubmit}
            className="w-full px-4 py-2 text-white bg-green-500 rounded-md hover:bg-green-600"
          >
            Adicionar
          </button>
        </form>
        <button
          type="button"
          onClick={onClose}
          className="absolute top-2 right-2 text-gray-500 hover:text-gray-700"
        >
          Fechar
        </button>
      </div>
    </div>
  );
};

export default PointModal;
