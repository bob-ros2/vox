# Vox: Real-time Whisper Transcription Tool & Library

Vox is a powerful, real-time audio transcription tool and Python library that uses OpenAI's Whisper model. It continuously listens to a microphone, detects speech, transcribes it upon pauses, and forwards the resulting text to one or more configurable destinations using a highly extensible, pluggable output system.

The project is designed to be used in two primary ways:
1.  **As a standalone command-line tool**, easily deployed with Docker.
2.  **As a Python library**, allowing you to integrate real-time transcription directly into your own applications.

## Key Features

-   **Real-time Transcription**: Listens to an audio input and transcribes speech in near real-time.
-   **Dual Use**: Functions as both a ready-to-run CLI tool and an importable Python library.
-   **Voice Activity Detection**: Intelligently waits for pauses in speech before transcribing, improving accuracy and reducing unnecessary processing.
-   **Pluggable Output System**: Send transcribed text to the console, files, HTTP webhooks, ROS2 topics, and more—simultaneously.
-   **Fully Extensible**: Load your own custom output handlers from a local script file without modifying the core code.
-   **Highly Configurable**: Adjust silence thresholds, language, Whisper model size, and more via command-line arguments.
-   **Dockerized**: Comes with a `Dockerfile` and `docker-compose.yaml` for easy, cross-platform deployment.
-   **GPU Acceleration**: Natively supports NVIDIA GPU acceleration via Docker for significantly faster transcription.

---

## Installation

There are three ways to install Vox for local use, depending on your needs. For all methods, please first review the **Dependencies and Manual Installation** section below.

### Method 1: From PyPI (Recommended for Users)

This is the easiest and most standard way to get the latest stable version. It will automatically make the `vox` command available in your terminal.

```bash
pip install vox-scribe
```

### Method 2: From GitHub (Latest Development Version)

If you want the absolute latest features or fixes that haven't been published to PyPI yet, you can install directly from the GitHub repository.

```bash
pip install git+https://github.com/bob-ros2/vox
```

### Method 3: From Source (For Developers)

If you have cloned the repository and want to modify the code, install it in "editable" mode. This links the `vox` command to your source files, so your changes are reflected immediately.

```bash
# From the root of the cloned repository
pip install -e .
```

---

## 1. Using Vox as a Standalone Tool

This is the recommended method for end-users who want a running transcription service.

### Quick Start with Docker

#### System Requirements
-   Docker & Docker Compose
-   A working microphone accessible by the host system.
-   (Recommended) An NVIDIA GPU with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
-   Linux (recommended, for easy PulseAudio integration).

#### Setup Steps
1.  **Clone the Repository**
    ```bash
    git clone https://github.com/bob-ros2/vox
    cd vox
    ```

2.  **Create the Environment File**
    The Docker container needs your user ID to access the PulseAudio audio socket.
    ```bash
    echo -e "UID=$(id -u)\nGID=$(id -g)" > .env
    ```

3.  **Configure and Run**
    Modify the `command` in `docker-compose.yaml` to suit your needs, then build and run the container.
    ```bash
    docker-compose up --build
    ```
    The application will start, download the Whisper model on the first run, and begin listening. Speak into your microphone, and after you pause, the transcription will be sent to the configured handlers. Press `Ctrl+C` to stop.

---

## 2. Using Vox as a Python Library

You can import the `Transcriber` class to integrate real-time transcription into your own Python projects.

### Installation

Please follow one of the methods described in the **Installation** section above to install the library.

### Example Usage

```python
# my_app.py
from vox.core import Transcriber
import time

def handle_transcription(text):
    """This function will be called with each new transcription."""
    print(f"My application received: {text}")

# Configuration dictionary (see CLI arguments table for all options)
my_config = {
    "model": "base",
    "lang": "en",
    "threshold": 1000,
    "silence_threshold": 150,
    # ... other parameters
}

try:
    # Instantiate the transcriber with your config and callback
    transcriber = Transcriber(config=my_config, on_transcription_callback=handle_transcription)
    
    # Run the transcription loop (this is a blocking call)
    transcriber.run()

except KeyboardInterrupt:
    print("Stopping transcription.")
finally:
    if 'transcriber' in locals():
        transcriber.close()
```

---

## Dependencies and Manual Installation

While **Docker is the recommended, hassle-free method**, you can install Vox locally. This requires manually installing its dependencies.

### Core Dependencies
-   `openai-whisper`: The core transcription engine.
-   `pyaudio`: For accessing the microphone audio stream.
-   `numpy`: A dependency for audio data manipulation.
-   `torch`: The deep learning framework required by Whisper.

### System-Level Dependencies
-   **PortAudio**: `PyAudio` is a Python wrapper for the PortAudio library. You must install PortAudio on your system before `pip install pyaudio` will succeed.
    -   On Debian/Ubuntu: `sudo apt-get install portaudio19-dev`
    -   On macOS (with Homebrew): `brew install portaudio`

---

## Configuration & Usage Examples

All settings are controlled via command-line arguments. These examples can be run directly (e.g., `vox [args...]`) or adapted for the `command` section in `docker-compose.yaml`.

### Example 1: Basic English Transcription
This is the simplest way to start. It uses the small `base` model and automatically detects the language.

```bash
vox --model base --rms
```
-   `--model base`: Uses the fast but less accurate `base` model.
-   `--rms`: Shows the real-time audio volume, helping you tune the silence threshold.

### Example 2: High-Quality German Transcription
For more accurate results, use a larger model and specify the language.

```bash
vox --model medium --lang de
```
-   `--model medium`: Uses the larger, more accurate `medium` model.
-   `--lang de`: Sets the transcription language to German, improving accuracy and avoiding misidentification.

### Example 3: Multiple Outputs (Console and File)
Send transcriptions to both your terminal and a log file simultaneously.

```bash
vox --output-handlers stdout file
```
-   `--output-handlers stdout file`: Activates both the console and file handlers. Transcriptions will be printed and also appended to `transcriptions.txt`.

### Example 4: Webhook Integration for an API
Send every transcription to a web server using the `http_post` handler.

```bash
vox --model small --output-handlers http_post --http-post-url "http://localhost:8000/api/transcribe"
```
-   `--output-handlers http_post`: Activates the HTTP handler.
-   `--http-post-url "..."`: (Required for `http_post`) Specifies the endpoint to send data to.

### Example 5: Using a Custom Handler Script
Load a custom handler from a local file for ultimate flexibility.

```bash
vox --output-handlers stdout --custom-handler ./my_custom_logger.py
```
-   `--custom-handler ...`: Loads the `OutputHandler` class from the specified Python file and adds it to the list of active handlers.

---

## The Output Handler System

Vox can send transcriptions to multiple destinations at once.

### Built-in Handlers
-   **`stdout`**: Prints to the console.
-   **`file`**: Appends to `transcriptions.txt` with a timestamp.
-   **`http_post`**: Sends a POST request to a URL. Perfect for webhooks. (Requires `requests`)
-   **`ros2`**: Publishes to a ROS2 topic. Fails gracefully if ROS2 libraries are not installed.

### Loading a Custom Handler from a Script
This is the most powerful feature for custom integrations.

1.  **Create a Handler File** (e.g., `my_db_logger.py`) with an `OutputHandler` class.
    ```python
    # my_db_logger.py
    class OutputHandler:
        def __init__(self, **kwargs):
            print("My DB logger is ready!")
        def send(self, text):
            print(f"LOGGING TO DB: {text}")
        def close(self):
            print("Closing DB connection.")
    ```
2.  **Mount the Script in Docker** and provide the path to the `--custom-handler` argument.
    ```yaml
    # docker-compose.yaml
    services:
      listener:
        volumes:
          # ... other volumes
          - ./my_db_logger.py:/app/my_db_logger.py
        command: ["python", "-m", "vox.cli",
                  "--output-handlers", "stdout",
                  "--custom-handler", "/app/my_db_logger.py"
        
    ```

---
## CLI Arguments Reference Table

| Argument                    | Default    | Description                                                                         |
| --------------------------- | ---------- | ----------------------------------------------------------------------------------- |
| `-m`, `--model`             | `base`     | Whisper model (`tiny`, `base`, `small`, `medium`, `large`).                           |
| `-l`, `--lang`              | `None`     | Language code (`en`, `de`). `None` for auto-detection.                              |
| `-t`, `--threshold`         | `1000`     | Pause duration in ms to trigger transcription.                                      |
| `-s`, `--silence-threshold` | `100`      | RMS audio level below which is considered silence.                                  |
| `-d`, `--device`            | `None`     | Index of the audio input device.                                                    |
| `-c`, `--chunk-size`        | `1024`     | Samples per audio chunk. Lower is more responsive.                                  |
| `--model-dir`               | `None`     | Directory to store/load Whisper models.                                             |
| `-p`, `--pre-buffer`        | `5`        | Chunks to keep before speech starts to avoid clipped words.                         |
| `-w`, `--warmup`            | `2`        | Consecutive loud chunks to start recording.                                         |
-r`, `--rms`               | `False`    | Show real-time audio volume (RMS).                                                  |
| `--output-handlers`         | `['stdout'` | A space-separated list of built-in handlers to use.                               |
| `--custom-handler`          | `None`     | Path to a Python script with a custom `OutputHandler`.                              |