"use client";

// Import necessary modules and components
import { useRef, useState } from "react";

// Export the MicrophoneComponent function component
export default function Talk({ emotion, setEmotion }) {
  // State variables to manage recording status, completion, and transcript

  const [audioBlob, setAudioBlob] = useState(null);
  const [isRecording, setIsRecording] = useState(false);
  const audioRef = useRef(null);

  const [audioSrc, setAudioSrc] = useState(null);
  const getAudio = async () => {
    let chunks = [];
    let recorder;

    try {
      // wait for the stream promise to resolve
      let stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      recorder = new MediaRecorder(stream);
      recorder.ondataavailable = (e) => {
        chunks.push(e.data);
        if (recorder.state === "inactive") {
          let blob = new Blob(chunks, { type: "audio/mp3" }); // Change type to "audio/mp3"
          setAudioBlob(blob); // Store the blob in state
          setIsRecording(false); // Update recording status
          saveAudioToEndpoint(blob);
        }
      };
      recorder.start();
      setIsRecording(true); // Update recording status
      setTimeout(() => {
        recorder.stop();
      }, 3000);
    } catch (e) {
      console.log("error getting stream", e);
      setIsRecording(false); // Update recording status in case of an error
    }
  };

  async function encodeAudioToBase64(blob) {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
  
      reader.onloadend = () => {
        resolve(reader.result.split(",")[1]); // Extracting the Base64 data (skip the data:image/png;base64, part)
      };
  
      reader.onerror = reject;
      reader.readAsDataURL(blob);
    });
  }

  // Send the MP3 file to your endpoint
  const saveAudioToEndpoint = async (blob) => {
    let exclamations = ["hmm", "pensada", "ponderar", "possibilidades", "ver"];
    let chosen = exclamations[Math.floor(Math.random() * exclamations.length)];
    setAudioSrc(`https://d17sdup6iumur7.cloudfront.net/exclamations/${chosen}.mp3`);
    const endpointUrl = process.env.NEXT_PUBLIC_BACKEND + "/speak"; // Replace with your actual endpoint URL


    try {
      const base64Data = await encodeAudioToBase64(blob);

      const dataToSend = {
        "audioData": String(base64Data)
      }

      console.log(dataToSend);

      const response = await fetch(endpointUrl, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(dataToSend),
      });


      if (response.ok) {
        console.log("Audio file sent successfully");
        const data = await response.json();
        console.log(data)
        if (data.emotion.includes("happy")) {
          setEmotion("happy");
        } else if (data.emotion.includes("sad")) {
          setEmotion("sad");
        } else if (data.emotion.includes("superhappy")) {
          setEmotion("superhappy");
        } else if (data.emotion.includes("neutral")) {
          setEmotion("neutral");
        }

        setAudioSrc(`data:audio/mp3;base64,${data.base64Audio}`);
      } else {
        console.error("Failed to send audio file");
      }
    } catch (error) {
      console.error("Error sending audio file", error);
    }
  };

  // Render the microphone component with appropriate UI based on recording state
  return (
    <div className="flex items-end justify-center h-screen w-full z-10">
      <div className="flex flex-col items-center justify-center">
        <button
          className={`bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded ${
            isRecording ? "opacity-50 cursor-not-allowed" : ""
          }`}
          onClick={getAudio}
          disabled={isRecording}
        >
          {isRecording ? "Recording..." : "Record"}
        </button>
        {audioBlob && (
          <div className="mt-2">
            {audioSrc && (
              <audio
                autoPlay={true}
                src={audioSrc}
                ref={audioRef}
                className="hidden"
              />
            )}
          </div>
        )}
      </div>
    </div>
  );
}
