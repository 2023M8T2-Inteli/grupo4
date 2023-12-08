const express = require("express");
const router = express.Router();
const multer = require("multer");
const speech = require("@google-cloud/speech");
const speechClient = new speech.SpeechClient();
const dotenv = require("dotenv");
dotenv.config();

const storage = multer.memoryStorage();
const upload = multer({ storage: storage });

router.post("/speak", upload.single("audioFile"), async (req, res) => {
  try {
    const audioBuffer = req.file.buffer;

    const config = {
      encoding: "MP3",
      sampleRateHertz: 16000,
      languageCode: "en-US",
    };

    const request = {
      audio: {
        content: audioBuffer,
      },
      config: config,
    };

    // Detects speech in the MP3 file
    const [response] = await speechClient.recognize(request);

    const transcription = response.results
      .map((result) => result.alternatives[0].transcript)
      .join("\n");

    console.log(`Transcription: ${transcription}`);

    res.status(200).json({
      message: "Audio file received, transcribed, and saved successfully",
      transcription,
    });
  } catch (error) {
    console.error("Error processing audio:", error);
    res.status(500).json({ error: "Internal server error" });
  }
});

module.exports = router;
