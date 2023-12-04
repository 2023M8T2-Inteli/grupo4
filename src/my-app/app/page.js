import React from "react";
import Sidebar from "./components/Sidebar";
import Now from "./components/Now";
import Fila from "./components/Fila";
import History from "./components/History";

const page = () => {
  return (
    <div className="h-screen w-full flex">
      <Sidebar />
      <div className="flex flex-col w-full">
        <div className="flex gap-4 p-4 h-1/2 justify-between w-full">
          <Now />
          <Fila />
        </div>
        <History/>
      </div>
    </div>
  );
};

export default page;
