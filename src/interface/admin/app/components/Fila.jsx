"use client";
import formatTime from "../utils/formatTime";
const Fila = ({queue}) => {

  return (
    <div className="border-lg shadow-md w-full h-full overflow-y-auto border-gray-100 border-[2px] rounded-md p-4 text-sm">
      <h1 className="text-2xl font-semibold mb-4">Na fila</h1>

      <table className="min-w-full border border-gray-300">
        <thead>
          <tr className="bg-gray-100 text-center">
            <th className="py-2 px-4 border-b">Item</th>
            <th className="py-2 px-4 border-b">Destino</th>
            <th className="py-2 px-4 border-b">Requisitante</th>
            <th className="py-2 px-4 border-b">Horário</th>
          </tr>
        </thead>
        <tbody>
          {queue.map((item) => (
            <tr key={item?.id} className="border-b text-center">
              <td className="py-2 px-4">{item?.tool?.name}</td>
              <td className="py-2 px-4">{item?.point?.name}</td>
              <td className="py-2 px-4">
                <span>{item?.user?.name}</span>
              </td>
              <td className="py-2 px-4">{formatTime(item.createdAt)}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default Fila;
