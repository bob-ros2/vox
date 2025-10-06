import pyaudio
import numpy as np
import whisper
import logging
from collections import deque

class Transcriber:

    def __init__(self, config, on_transcription_callback):
        """
        Initializes the Transcriber.

        Args:
            on_transcription_callback (callable): A function to call with the transcribed text.
            config (dict): A dictionary containing configuration parameters. Expected keys are:
                - model (str): Name of the Whisper model (e.g., 'base', 'small', 'medium').
                - model_dir (str, optional): Directory to store/load Whisper models from.
                - lang (str, optional): Language for transcription (e.g., 'en', 'de'). None for auto-detection.
                - device (int, optional): Index of the audio input device. None for default.
                - threshold (int): Time in milliseconds a pause must last to end the recording (e.g., 1000).
                - silence_threshold (int): RMS audio level below which is considered silence (e.g., 100).
                - chunk_size (int): Number of samples per audio chunk (e.g., 1024).
                - pre_buffer (int): Number of audio chunks to keep before speech is detected (e.g., 5).
                - warmup (int): Number of consecutive loud chunks to start recording (e.g., 2).
                - channels (int): Number of audio channels (e.g., 1).
                - rate (int): Audio sample rate in Hz (e.g., 16000).
                - rms (bool): If True, prints real-time audio volume.
        """
        self.config = config
        self.on_transcription = on_transcription_callback

        MODEL = self.config.get('model', 'base')
        logging.info(f"Loading Whisper model: {MODEL}...")

        self.model = whisper.load_model(MODEL, 
            download_root=self.config.get('model_dir', None))

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=self.config.get('channels', 2),
            rate=self.config.get('rate', 16000),
            input=True,
            frames_per_buffer=self.config.get('chunk_size', 1024),
            input_device_index=self.config.get('device', None)
        )
        logging.info("Ready. Start speaking to begin transcription...")

    def run(self):
        """Starts the main transcription loop."""

        # --- Audio Parameters ---
        CHUNK = self.config.get('chunk_size', 1024)
        RATE = self.config.get('rate', 16000)

        # --- Main Loop for Real-time Processing ---
        recording = False
        silence_counter = 0
        audio_buffer = []
        pre_buffer = deque(
            maxlen=self.config.get('pre_buffer', 2))  
        warmup_counter = 0

        while True:
            try:
                data = self.stream.read(CHUNK)
                audio_chunk = np.frombuffer(data, dtype=np.int16)
                rms = np.sqrt(np.mean(np.square(audio_chunk)))
                # Print the value (overwrite the same line)
                self.config.get('rms', False) and print(f"\rRMS: {rms:6.2f} ", end="", flush=True)

                # Check if a recording is currently active
                if recording:
                    audio_buffer.append(data)
                    # Check for silence
                    if rms < self.config.get('silence_threshold', 15) :    
                        silence_counter += 1
                    else:
                        silence_counter = 0

                    # If silence was long enough, transcribe and reset
                    # The calculation converts the threshold from ms to number of chunks
                    if silence_counter > (self.config.get('threshold',1000) / (CHUNK / RATE * 1000)):
                        logging.info("--- Pause detected. Transcribing...")
                        audio_data = np.frombuffer(
                            b''.join(audio_buffer), 
                            dtype=np.int16).astype(np.float32) / 32768.0

                        # Transcribe with the specified language, if provided
                        result = self.model.transcribe(
                            audio_data, 
                            language=self.config.get('lang', None))

                        logging.info(f"Transcription: {result['text']}")

                        # Use the handlers to send the output
                        text = result['text'].strip()
                        if text:
                            self.on_transcription(text)

                        # Reset the buffer and stop recording
                        recording = False
                        audio_buffer = []
                        silence_counter = 0
                        warmup_counter = 0 
                        logging.info("Waiting for new speech...")

                else:
                    # Wait until speech begins
                    if rms > self.config.get('silence_threshold', 100) * 2: 
                        warmup_counter += 1
                    else:
                        warmup_counter = 0 # Reset if there is a dip in volume

                    # If we have enough consecutive loud chunks, start recording
                    if warmup_counter >= self.config.get('warmup', 2):
                        logging.info("--- Speech detected. Recording started...")
                        recording = True
                        # Add the pre-buffer to the main audio buffer to capture the start of speech
                        audio_buffer.extend(list(pre_buffer))
                        warmup_counter = 0 # Reset for the next detection

            except KeyboardInterrupt:
                break

    def close(self):
        """Cleans up resources."""
        logging.info("Closing transcriber...")
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
