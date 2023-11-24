import { useEffect, useState, useRef } from 'react';

const AudioPlayer = () => {
  const [audioSrc, setAudioSrc] = useState(null);
  const audioRef = useRef(null);

  useEffect(() => {
    const fetchAudio = async () => {
      try {
        const response = await fetch('http://localhost:5000/audio');
        if (!response.ok) {
          throw new Error('Failed to fetch audio');
        }

        const data = await response.json();
        setAudioSrc(`data:audio/mp3;base64,${data.speech}`);
      } catch (error) {
        console.error('Fetch Audio Error:', error.message);
      }
    };

    const fetchInterval = setInterval(fetchAudio, 5000); // Fetch every 10 seconds

    // Initial fetch
    fetchAudio();

    // Cleanup interval on component unmount
    return () => clearInterval(fetchInterval);
  }, []);

  return (
    <div>
      <h1>Audio Player</h1>
      {audioSrc && <audio autoPlay={true} src={audioSrc} ref={audioRef} />}
    </div>
  );
};

export default AudioPlayer;
