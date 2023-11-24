import { useEffect, useState, useRef } from 'react';

const AudioPlayer = () => {
  const [eventSource, setEventSource] = useState(null);
  const [audioSrc, setAudioSrc] = useState(null);
  const audioRef = useRef(null);

  useEffect(() => {
    const source = new EventSource('http://localhost:5000/sse');
    console.log('running')

    source.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setAudioSrc(`data:audio/mp3;base64,${data.audio}`);
    };

    source.onerror = (error) => {
      console.error('SSE Error:', error);
      source.close();
    };

    setEventSource(source);

    return () => {
      if (source) {
        source.close();
      }
    };
  }, []);

  useEffect(() => {
    // Simulate a click on the audio element when the audio source is set
    if (audioSrc && audioRef.current) {
      audioRef.current.click();
    }
  }, [audioSrc]);

  return (
    <div>
      <h1>Audio Player</h1>
      {audioSrc && <audio autoPlay={true} src={audioSrc} ref={audioRef} />}
    </div>
  );
};

export default AudioPlayer;
