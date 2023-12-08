"use client";

import Sidebar from "../components/Sidebar";
import Points from "../components/Points";

const page = () => {
  return (
    <div className="h-screen w-full flex">
      <Sidebar />
      <div className="flex flex-col w-full">
        <Points/>
      </div>
    </div>
  );
};

export default page;
