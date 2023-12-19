"use client";

import Sidebar from "../components/Sidebar";
import Users from "../components/Users";

const page = () => {
  return (
    <div className="h-screen w-full flex">
      <Sidebar />
      <div className="flex flex-col w-full">
        <Users/>
      </div>
    </div>
  );
};

export default page;
