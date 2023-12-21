"use client";
import React from "react";
import Sidebar from "../components/Sidebar";
import Tools from "../components/Tools";

const page = () => {
  return (
    <div className="h-screen w-full flex">
      <Sidebar />
      <div className="flex flex-col w-full">
        <Tools/>
      </div>
    </div>
  );
};

export default page;
