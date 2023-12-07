const Now = ({now}) => {
  return (
    <div className="shadow-md border-gray-100 border-[2px] rounded-md flex flex-col justify-between p-4 w-full h-full">
      <span className="flex items-center justify-between">
        <h1 className="text-2xl font-semibold">Agora</h1>
        <h3>Bateria: 50%</h3>
      </span>
      <span>
        <h3 className="text-gray-400 font-semibold text-sm">DESTINO</h3>
        <h1 className="text-3xl text-bold">{now?.point?.name}</h1>
      </span>

      <span className="flex justify-between">
        <h3>{now?.tool?.name}</h3>
        <h3>{now?.user?.name}</h3>
      </span>
      <button className="w-full border-red-400 border-[1px] text-red-400 rounded-md">
        PARAR
      </button>
    </div>
  );
};

export default Now;
