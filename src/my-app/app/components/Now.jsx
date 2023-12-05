const Now = () => {
  return (
    <div className="shadow-md border-gray-100 border-[2px] rounded-md flex flex-col justify-between p-4 w-full">
      <span className="flex items-center justify-between">
        <h1 className="text-2xl font-semibold">Agora</h1>
        <h3>Bateria: 50%</h3>
      </span>
      <span>
        <h3 className="text-gray-400 font-semibold text-sm">DESTINO</h3>
        <h1 className="text-3xl text-bold">Zé Delivery</h1>
      </span>

      <span className="flex justify-between">
        <h3>10m estimados</h3>
        <h3>Marcos Aurélio</h3>
      </span>
      <button className="w-full border-red-400 border-[1px] text-red-400 rounded-md">
        PARAR
      </button>
    </div>
  );
};

export default Now;
