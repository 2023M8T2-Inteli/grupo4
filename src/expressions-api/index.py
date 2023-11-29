from flask import Flask, request, Response, jsonify
from flask_cors import CORS
from threading import Lock
import json
import base64
import os
import dotenv

dotenv.load_dotenv()

app = Flask(__name__)
CORS(app)

from google.cloud import texttospeech

def synthesize_text_base64(text, language_code="pt-BR", voice_name='pt-BR-Wavenet-B', pitch=13):
    # Instantiates a client
    client = texttospeech.TextToSpeechClient()

    # Set the text input to be synthesized
    synthesis_input = texttospeech.SynthesisInput(text=text)

    # Specify the voice parameters
    voice = texttospeech.VoiceSelectionParams(
        language_code=language_code,
        name=voice_name,
    )

    # Specify the type of audio file you want returned
    audio_config = texttospeech.AudioConfig(
        pitch=pitch,
        audio_encoding=texttospeech.AudioEncoding.LINEAR16
    )

    response = client.synthesize_speech(
        input=synthesis_input, voice=voice, audio_config=audio_config
    )

    # Encode the audio content to base64
    audio_base64 = base64.b64encode(response.audio_content).decode('utf-8')

    return audio_base64

# Variable to store data
stored_data = {
    'speech': 'Bom dia, eu sou o Raphinha, o robô autônomo da Cervejaria do Futuro, na Ambev.',
    'emotion': 'happy',
}

@app.route('/audio')
def get_audio():
    current_text = stored_data['speech']
    audio_base64 = synthesize_text_base64(current_text)
    
    response_data = {
        'speech': audio_base64,
        'emotion': stored_data['emotion'],
    }

    print(response_data)
    return jsonify(response_data)

# Handle POST requests to update data
@app.route('/update', methods=['POST'])
def update():
    new_data = request.json

    stored_data.update({
        **new_data,
        'timestamp': new_data['timestamp'] if 'timestamp' in new_data else '',
    })

    return jsonify({'message': 'Data updated successfully'}), 200


# Start the server
if __name__ == '__main__':
    PORT = int(os.environ.get('PORT', 5000))
    app.run(port=PORT)
