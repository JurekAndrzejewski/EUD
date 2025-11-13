"use client";


import { useState } from "react";


export default function Home() {
  const [message, setMessage] = useState<string>("");
  const [loading, setLoading] = useState(false);


  const handleClick = async () => {
    try {
      setLoading(true);
      const res = await fetch("http://YOUR-NODERED-URL/endpoint");
      const data = await res.text();
      setMessage(data);
    } catch (err) {
      setMessage("Error contacting Node-RED");
    } finally {
      setLoading(false);
    }
  };


  return (
    <div className="flex items-center justify-center h-screen bg-gray-100">
      <div className="p-6 rounded-2xl shadow-lg bg-white w-80 text-center space-y-4">
        <button
          onClick={handleClick}
          className="px-4 py-2 rounded-2xl shadow bg-blue-500 text-white hover:bg-blue-600 transition"
        >
          {loading ? "Loading..." : "Query Node-RED"}
        </button>
        {message && (
          <div className="mt-4 p-3 bg-blue-500 rounded-xl text-sm break-words">
            {message}
          </div>
        )}
      </div>
    </div>
  );
}