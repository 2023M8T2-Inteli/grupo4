import whisper
import sounddevice as sd
import numpy as np
import wave

def record_audio(duration=10, sample_rate=44100):
    print("Recording...")

    audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=2, dtype=np.int16)
    sd.wait()

    with wave.open('speech.mp3', 'wb') as wf:
        wf.setnchannels(2)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(audio_data.tobytes())

    print(f"Audio recorded and saved to speech.mp3")
    return 'speech.mp3'

model = whisper.load_model("base")
result = model.transcribe(record_audio())
print(result["text"])