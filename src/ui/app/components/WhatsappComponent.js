// WhatsappButton.js
import { FaWhatsapp } from "react-icons/fa";

export default function WhatsappButton({ onClick }) {
  return (
    <span className="absolute top-2 right-2 z-10">
      <div
        className="rounded-full border-2 w-14 h-14 border-lime-700 text-white flex items-center justify-center text-2xl"
        onClick={onClick}
      >
        <FaWhatsapp />
      </div>
    </span>
  );
}
