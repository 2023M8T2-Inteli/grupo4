from fastapi import FastAPI, HTTPException, Depends, File, UploadFile, Form, APIRouter
from fastapi.responses import JSONResponse
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel
from typing import Optional
import base64
import os
from google.cloud import texttospeech

router = APIRouter()

class TextToSpeechInput(BaseModel):
    text: str
    language_code: Optional[str] = "pt-BR"
    voice_name: Optional[str] = "pt-BR-Wavenet-B"
    pitch: Optional[int] = 13

def synthesize_text_base64(text, language_code="pt-BR", voice_name='pt-BR-Wavenet-B', pitch=13):
    client = texttospeech.TextToSpeechClient()
    input_text = texttospeech.SynthesisInput(text=text)
    voice = texttospeech.VoiceSelectionParams(
        language_code=language_code, name=voice_name
    )
    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3, pitch=pitch
    )
    response = client.synthesize_speech(
        input=input_text, voice=voice, audio_config=audio_config
    )
    return base64.b64encode(response.audio_content).decode("utf-8")

@router.post("/tts")
def text_to_speech(input_data: TextToSpeechInput):
    audio_base64 = synthesize_text_base64(
        input_data.text, input_data.language_code, input_data.voice_name, input_data.pitch
    )
    return JSONResponse(content={"speech": audio_base64})
