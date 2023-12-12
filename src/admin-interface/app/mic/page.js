"use client";

// Import necessary modules and components
import { useRef, useState } from "react";

// Export the MicrophoneComponent function component
export default function MicrophoneComponent() {
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

      recorder.start(5000);
      setIsRecording(true); // Update recording status

      setTimeout(() => {
        recorder.stop();
      }, 2000);
    } catch (e) {
      console.log("error getting stream", e);
      setIsRecording(false); // Update recording status in case of an error
    }
  };

  // Send the MP3 file to your endpoint
  const saveAudioToEndpoint = async (blob) => {
    const endpointUrl = "http://localhost:5000/speech/speak"; // Replace with your actual endpoint URL

    try {
      const formData = new FormData();
      formData.append("audioFile", blob, "recorded_audio.mp3");

      const response = await fetch(endpointUrl, {
        method: "POST",
        body: formData,
      });

      if (response.ok) {
        console.log("Audio file sent successfully");
        const data = await response.json();
        console.log(data);
        setAudioSrc(`data:audio/mp3;base64,${data.base64Audio}`);
      } else {
        console.error("Failed to send audio file");
      }
    } catch (error) {
      console.error("Error sending audio file", error);
    }
  };
  const playAudio = () => {
    if (audioBlob) {
      // Create an audio element
      const audioElement = new Audio();
      // Set the source of the audio element to the recorded blob
      audioElement.src = URL.createObjectURL(audioBlob);

      // Ensure the audio is loaded before playing
      audioElement.addEventListener("loadedmetadata", () => {
        audioElement.play();
      });

      // Clean up resources after playing
      audioElement.addEventListener("ended", () => {
        URL.revokeObjectURL(audioElement.src);
      });
    }
  };

  // Render the microphone component with appropriate UI based on recording state
  return (
    <div className="flex items-center justify-center h-screen w-full">
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
            <button
              className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded"
              onClick={playAudio}
            >
              Play
            </button>
            {audioSrc && <audio autoPlay={true} src={audioSrc} ref={audioRef} className='hidden' />}
          </div>
        )}
      </div>
    </div>
  );
}