# Package [VOX](https://github.com/bob-ros2/vox) Real-time Transcription

Vox is a powerful, real-time audio transcription tool and Python library that uses OpenAI's Whisper model. It continuously listens to a microphone, detects speech, transcribes it upon pauses, and forwards the resulting text to one or more configurable destinations using a highly extensible, pluggable output system.

The project is designed to be used in two primary ways:
1.  **As a standalone command-line tool**, easily deployed with Docker.
2.  **As a Python library**, allowing you to integrate real-time transcription directly into your own applications.

## Key Features

-   **Real-time Transcription**: Listens to an audio input and transcribes speech in near real-time.
-   **Dual Use**: Functions as both a ready-to-run CLI tool and an importable Python library.
-   **Voice Activity Detection**: Intelligently waits for pauses in speech before transcribing, improving accuracy and reducing unnecessary processing.
-   **Pluggable Output System**: Send transcribed text to the console, files, HTTP webhooks, ROS2 topics, and more simultaneously.
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
pip install vox-real
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

#### Option 1: Use the Pre-built Image from GHCR (Recommended for Users)

This is the fastest way to get started. It pulls the ready-to-use Docker image directly from the GitHub Container Registry, skipping the local build process.

1.  **Create a Local Directory for Models**
    Whisper models will be downloaded and cached here to persist between container runs.
    ```bash
    mkdir -p ./models
    ```

2.  **Pull and Run the Image**
    Copy and paste the command below into your terminal. It replicates the setup from `docker-compose.yaml`, including audio and optional GPU support.

    ```bash
    docker run -it --rm \
      --gpus all \
      -v "${XDG_RUNTIME_DIR}/pulse/native:/run/pulse/native" \
      -v "/etc/machine-id:/etc/machine-id:ro" \
      -v "./models:/models" \
      ghcr.io/bob-ros2/vox/vox-real:latest \
      vox --model-dir /models --model base --lang en --rms
    ```
    -   `--gpus all`: **(Optional)** Provides NVIDIA GPU acceleration. Remove this flag if you don't have an NVIDIA GPU or the NVIDIA Container Toolkit.
    -   The `-v` flags are essential for mounting your system's audio socket and the local `models` directory into the container.
    -   You can customize the transcription arguments (like `--model`, `--lang`, etc.) at the end of the command.

#### Option 2: Build from Source (For Developers)
This method builds the Docker image locally. It's best if you plan to modify the code.

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
    transcriber = Transcriber(config=my_config, on_transcription_cb=handle_transcription)
    
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

While **Docker is the recommended, hassle-free method**, you can install Vox locally. This requires some system-level setup.

1.  **Install PortAudio**: `pyaudio` requires the PortAudio C library.
    -   **Debian/Ubuntu**: `sudo apt-get install portaudio19-dev`
    -   **macOS (Homebrew)**: `brew install portaudio`

2.  **Install Python Dependencies**: It is highly recommended to use a virtual environment.
    ```bash
    pip install vox-real
    ```

**Note on `torch`**: Whisper depends on PyTorch (`torch`), which can be complex to install, especially for GPU support. If you encounter issues, we strongly recommend following the official instructions at [pytorch.org](https://pytorch.org/get-started/locally/) or using the provided Docker setup.


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
## Troubleshooting

### Audio Quality Issues (Slow/Fast Audio)
If you experience poor transcription quality or the audio sounds slowed down or sped up, it is likely due to a sample rate mismatch.
- **Symptom**: Transcription is gibberish or non-existent, and if you listen to the raw audio, it sounds pitch-shifted.
- **Cause**: The Whisper model expects audio at 16000 Hz. If your input device or the application is configured for a different rate (e.g., 44100 Hz or 48000 Hz) without proper resampling, the audio data will be misinterpreted.
- **Solution**: Ensure the `--rate` argument matches what Whisper expects (default is 16000 Hz). If you are using a custom setup or specific hardware that requires a different rate, you may need to force the rate using `--rate 16000`.

### ALSA lib Errors at Startup
You may see various `ALSA lib` errors (e.g., `unable to open slave`, `Unknown PCM`, `Cannot open device`) when `vox` starts up. These are generated by the underlying audio library (`pyaudio`/PortAudio) as it probes for available devices. **These errors are usually harmless and do not impact functionality.** If the application prints "Ready. Start speaking...", it is working correctly.

### Windows Support
Vox is compatible with Windows.
- **ALSA Errors**: You will not see ALSA errors on Windows, as ALSA is Linux-specific.
- **Dependencies**: You may need to install `ffmpeg` manually and add it to your PATH, as it is required by Whisper.
- **ROS2**: The `ros2` handler works on Windows if a compatible ROS2 distribution (e.g., Humble) is installed and sourced.

---
## CLI Arguments Reference Table
| Argument                    | Default    | Description                                                                         |
| --------------------------- | ---------- | ----------------------------------------------------------------------------------- |
| `-m`, `--model`             | `base`     | Whisper model to use (`tiny`, `base`, `small`, `medium`, `large`).                           |
| `-g`, `--lang`              | `None`     | Language code (`en`, `de`). `None` for auto-detection.                              |
| `-t`, `--threshold`         | `1000`     | Pause duration in ms to trigger transcription.                                      |
| `-s`, `--silence-threshold` | `20`       | RMS audio level below which is considered silence.                                  |
| `-d`, `--device`            | `None`     | Index of the audio input device.                                                    |
| `-c`, `--chunk-size`        | `1024`     | Samples per audio chunk. Lower is more responsive.                                  |
| `--model-dir`               | `None`     | Directory to store/load Whisper models.                                             |
| `-p`, `--pre-buffer`        | `5`        | Chunks to keep before speech starts to avoid clipped words.                         |
-w`, `--warmup`            | `2`        | Consecutive loud chunks to start recording.                                         |
| `-r`, `--rms`               | `False`    | Show real-time audio volume (RMS).                                                  |
| `--channels`                | `1`        | Number of audio channels.                                                           |
| `--rate`                    | `16000`    | Audio sample rate in Hz.                                                            |
| `--whisper-device`          | `None`     | Device to use for Whisper inference (`cpu`, `cuda`, `cuda:0`). `None` for auto-detect.|
| `--output-handlers`         | `stdout`| A space-separated list of built-in handlers to use. Built in: `stdout`, `file`, `ros2`                              |
| `--custom-handler`          | `None`     | Path to a Python script with a custom `OutputHandler`.                              |
| `-l`, `--list-devices`      | `False`    | List available audio input devices and exit.                                        |
| `--log-level`               | `info`     | Set the logging level. Use warning or lower to supress most of the logging. (`debug`, `info`, `warning`, `error`).                        |

### Handler-Specific Arguments
These arguments are only used when the corresponding handler is enabled via `--output-handlers`.

#### `file` handler
| Argument | Default | Description |
|---|---|---|
| `--file-name` | `transcriptions.txt` | The name of the file to save transcriptions to. |
| `--file-no-timestamp` | `False` | If set, timestamps will not be added to the output file. |

#### `http_post` handler
| Argument | Default | Description |
|---|---|---|
| `--http-post-url` | (Required) | The URL to which the transcription text will be POSTed. |
| `--http-post-format` | `json` | The format of the POST request body (`json` or `raw`). |
| `--http-post-header` | `None` | A JSON string of key/value pairs for request headers (e.g., `'{"Authorization": "Bearer YOUR_TOKEN"}'`). |

#### `ros2` handler
For ROS (Robot Operating System) users.
| Argument | Default | Description |
|---|---|---|
| `--ros2-topic` | `transcription` | The ROS2 topic to publish transcriptions to. |

## Robust Configuration Examples

Here are some examples of how to configure `vox` for different scenarios, building on the basic command:
`python -m vox.cli --model-dir ./models --model medium --lang de --rms --output-handlers stdout ros2 --ros2-topic /you/llm_prompt`

### 1. Reducing Hallucinations (Repetitions)
If the model gets stuck repeating the same phrase or hallucinating silence, try increasing the temperature slightly and disabling conditioning on previous text.
```bash
python -m vox.cli ... --temperature 0.2 --no-condition-on-previous-text
```

### 2. Improving Context and Style
Use `--initial-prompt` to tell the model about the context (e.g., technical terms, specific spelling) or the desired style.
```bash
python -m vox.cli ... --initial-prompt "Dies ist ein Transkript über Robotik und ROS2."
```

### 3. Maximizing Accuracy (Slower)
Increase the `beam-size` to explore more decoding paths. This will increase latency but may improve accuracy for complex sentences.
```bash
python -m vox.cli ... --beam-size 10
```

### 4. Low-Resource / Compatibility Mode
If you encounter errors related to FP16 on older hardware, force FP32.
```bash
python -m vox.cli ... --no-fp16
```
