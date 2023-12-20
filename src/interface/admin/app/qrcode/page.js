"use client";

import Sidebar from "../components/Sidebar";
import QRCodeComponent from "../components/QRCode";

const page = () => {
  return (
    <div className="h-screen w-full flex">
      <Sidebar />
      <div className="flex flex-col w-full">
        <QRCodeComponent/>
      </div>
    </div>
  );
};

export default page;