"use client";

import Sidebar from "../components/Sidebar";
import QRCode from "../components/QRCode";

const page = () => {
  return (
    <div className="h-screen w-full flex">
      <Sidebar />
      <div className="flex flex-col w-full">
        <QRCode/>
      </div>
    </div>
  );
};

export default page;