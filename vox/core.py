import pyaudio
import numpy as np
import whisper
import logging
from collections import deque
import threading
import queue
import sys

class Transcriber:

    def __init__(self, config, on_transcription_cb):
        """
        Initializes the Transcriber.

        Args:
            on_transcription_cb (callable): A function to call with the transcribed text.
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
                - rate (int): Audio sample rate in Hz (e.g., 16000).
                - rms (bool): If True, prints real-time audio volume.
                - initial_prompt (str): Optional context.
                - temperature (float): Sampling temperature.
                - beam_size (int): Beam size.
                - patience (float): Beam search patience.
                - length_penalty (float): Length penalty.
                - no_condition_on_previous_text (bool): If True, disables conditioning.
                - no_fp16 (bool): If True, forces FP32.
        """
        self.config = config
        self.on_transcription = on_transcription_cb
        self.recording_enabled = True
        self.transcription_queue = queue.Queue()

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

        # Start a separate thread to listen for the toggle command ONLY if interactive.
        if not self.config.get('non_interactive', False) and sys.stdin.isatty():
            self.toggle_thread = threading.Thread(target=self._listen_for_toggle)
            self.toggle_thread.daemon = True
            self.toggle_thread.start()
            logging.info("Press ENTER to toggle recording on/off.")
        else:
            self.toggle_thread = None

        # Start the single, dedicated transcription worker thread
        self.transcription_processor_thread = threading.Thread(
            target=self._transcription_processor)
        self.transcription_processor_thread.daemon = True
        self.transcription_processor_thread.start()
        
        logging.info("Ready. Start speaking to begin transcription...")

    def _listen_for_toggle(self):
        """Listens for ENTER in stdin to toggle recording."""
        for line in sys.stdin:
            self.recording_enabled = not self.recording_enabled
            status = "ON" if self.recording_enabled else "OFF"
            print(f"\r--- Recording toggled {status} ---")

    def _transcription_processor(self):
        """
        Runs in a dedicated thread.
        Pulls audio data from the queue and transcribes it.
        """
        while True:
            try:
                # Block until an item is available
                audio_data = self.transcription_queue.get()

                # A None item is a sentinel to stop the thread
                if audio_data is None:
                    break

                # Transcribe with the specified language, if provided
                result = self.model.transcribe(
                    audio_data,
                    language=self.config.get('lang', None),
                    initial_prompt=self.config.get('initial_prompt', None),
                    temperature=self.config.get('temperature', 0.0),
                    beam_size=self.config.get('beam_size', None),
                    patience=self.config.get('patience', None),
                    length_penalty=self.config.get('length_penalty', None),
                    condition_on_previous_text=not self.config.get('no_condition_on_previous_text', False),
                    fp16=not self.config.get('no_fp16', False)
                )

                logging.info(f"Transcription: {result['text']}")

                # Use the handlers to send the output
                text = result['text'].strip()
                if text:
                    self.on_transcription(text)

            except Exception as e:
                logging.error(f"Error during transcription: {e}")
            finally:
                # Ensure task_done is called even if an error occurs
                self.transcription_queue.task_done()

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

                if not self.recording_enabled:
                    # BUGFIX: The pre_buffer needs to be populated even when not recording.
                    pre_buffer.append(data)
                    if recording:
                        # Reset state if recording was in progress
                        recording = False
                        audio_buffer = []
                        silence_counter = 0
                        warmup_counter = 0
                        logging.info("Recording paused. Waiting for new speech...")
                    pre_buffer.clear() # Also clear pre_buffer to avoid old audio
                    continue

                audio_chunk = np.frombuffer(data, dtype=np.int16)

                # Handle multi-channel audio by reshaping and averaging
                num_channels = self.config.get('channels', 1)
                if num_channels > 1:
                    # Reshape the array to separate channels, then average them
                    audio_chunk = audio_chunk.reshape(-1, num_channels)
                    audio_chunk = audio_chunk.mean(axis=1)

                # Add a small epsilon to prevent sqrt of zero
                rms = np.sqrt(np.mean(np.square(audio_chunk.astype(np.float64))) + 1e-9)

                # Print the value (overwrite the same line)
                self.config.get('rms', False) and print(f"\rRMS: {rms:5.2f} ", end="", flush=True)

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
                        logging.info("--- Pause detected. Queueing audio for transcription...")
                        
                        # Prepare audio data
                        audio_data_bytes = b''.join(audio_buffer)
                        audio_data_raw = np.frombuffer(audio_data_bytes, dtype=np.int16)

                        # BUGFIX: Convert multi-channel audio to mono before transcription
                        num_channels = self.config.get('channels', 1)
                        if num_channels > 1:
                            audio_data_raw = audio_data_raw.reshape(-1, num_channels)
                            audio_data_raw = audio_data_raw.mean(axis=1)

                        audio_data = audio_data_raw.astype(np.float32) / 32768.0

                        # Put the prepared audio data into the queue
                        self.transcription_queue.put(audio_data)

                        # Reset state and stop recording
                        recording = False
                        audio_buffer = []
                        silence_counter = 0
                        warmup_counter = 0
                        logging.info("Waiting for new speech...")

                else:
                    # Wait until speech begins
                    if rms > self.config.get('silence_threshold', 15) * 2: # pseudo hysteresis
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
        
        # Signal the transcription thread to stop
        self.transcription_queue.put(None)
        # Wait for the transcription thread to finish
        self.transcription_processor_thread.join()

        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
