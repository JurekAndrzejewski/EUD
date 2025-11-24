// app/page.tsx (Next.js 13+ with App Router)
"use client";

import Link from "next/link";
import { useState } from "react";

export default function Home() {
  const [message, setMessage] = useState<string>("");
  const [loading, setLoading] = useState(false);

  const [open1, setOpen1] = useState(false);
  const [open2, setOpen2] = useState(false);
  const [open3, setOpen3] = useState(false);

  const handleClick = async () => {
    try {
      setLoading(true);
      const res = await fetch("http://127.0.0.1:1880/case1");
      const data = await res.text();
      setMessage(data);
    } catch (err) {
      setMessage("Error contacting Node-RED");
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      <header className="w-full flex justify-center bg-indigo-600 shadow-md py-4 fixed top-0 left-0 z-50">
        <nav className="flex space-x-6 text-lg font-medium">
          <Link href="/" className="hover:text-blue-600 transition">Home</Link>
          <Link href="/cases/one" className="font-bold underline hover:text-blue-600 transition">Case 1</Link>
          <Link href="/cases/two" className="hover:text-blue-600 transition">Case 2</Link>
          <Link href="/cases/three" className="hover:text-blue-600 transition">Case 3</Link>
        </nav>
      </header>

      <main className="pt-24 min-h-screen bg-blue-100 flex items-center justify-center">
        <div className="p-6 rounded-2xl shadow-lg bg-white w-96 text-center space-y-6">
          {/* Main Query Button */}
          <button
            onClick={handleClick}
            className="px-4 py-2 rounded-2xl shadow bg-blue-500 text-white hover:bg-blue-600 transition w-full"
          >
            {loading ? "Loading..." : "Start Robot Flow"}
          </button>

          {message && (
            <div className="mt-2 p-3 bg-blue-500 rounded-xl text-sm break-words text-white">
              {message}
            </div>
          )}

          {/* Placeholder Toggle Buttons (shown only after query) */}
          <div className={message ? "block" : "hidden"}>
            <div className="text-left space-y-4">
              <div>
                <button
                  onClick={() => setOpen1(!open1)}
                  className="w-full px-4 py-2 rounded-xl bg-blue-500 hover:bg-blue-600 transition"
                >
                  Why did the robot not pick up the second red cap?
                </button>
                {open1 && (
                  <div className="mt-2 p-3 bg-blue-400 rounded-xl">
                    The flow might not include a correct 'move to red cap 2' + 'grasp' sequence. Check that both actions are present and positioned before moving to the red bucket.
                  </div>
                )}
              </div>

              <div>
                <button
                  onClick={() => setOpen2(!open2)}
                  className="w-full px-4 py-2 rounded-xl bg-blue-500 hover:bg-blue-600 transition"
                >
                  Why did the robot skip one or more red caps?
                </button>
                {open2 && (
                  <div className="mt-2 p-3 bg-blue-400 rounded-xl">
                    Verify that all three red cap blocks exist in order: move - grasp - move to bucket - release. Missing or misplaced blocks will cause skipping.
                  </div>
                )}
              </div>

  
            </div>
          </div>
        </div>
      </main>
    </>
  );
}
