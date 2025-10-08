# Vox-Scribe: Real-time Whisper Transcription Tool & Library

**Vox-Scribe** is a powerful, real-time audio transcription tool and Python library that uses OpenAI's Whisper model. It continuously listens to a microphone, detects speech, and transcribes it upon pauses, making it ideal for a wide range of voice-activated applications.

This package can be used in two primary ways:
1.  **As a standalone command-line tool** (`vox`).
2.  **As a Python library** to integrate real-time transcription into your own applications.

> **Note**: This is the README for the PyPI package. For more advanced features, including **Docker support, ROS2 integration, and contribution guidelines**, please visit the full project repository on GitHub:
> - [https://github.com/bob-ros2/vox](https://github.com/bob-ros2/vox)


---

## Installation

First, ensure you have the necessary system-level dependencies (see the "Dependencies" section below). Then, you can install the package directly from PyPI. It is highly recommended to use a virtual environment.

```bash
pip install vox-scribe
```

This will make the `vox` command available in your terminal.

---

## Quick Start (Command-Line Tool)

Once installed, you can start transcribing from your default microphone with a simple command:

```bash
# Start with the small, fast 'base' model and show real-time audio volume
vox --model base --rms
```

Speak into your microphone. When you pause, the transcribed text will be printed to the console. Press `Ctrl+C` to stop.

---

## Key Features

-   **Real-time Transcription**: Listens to an audio input and transcribes speech in near real-time.
-   **Voice Activity Detection**: Intelligently waits for pauses in speech before transcribing.
-   **Dual Use**: Functions as both a ready-to-run CLI tool and an importable Python library.
-   **Highly Configurable**: Adjust silence thresholds, language, Whisper model size, and more.
-   **Pluggable Output System**: Send transcribed text to the console, files, or an HTTP webhook.

---

## Using Vox as a Python Library

You can easily integrate real-time transcription into your own projects by using the `Transcriber` class.

### Library Example

```python
# my_app.py
from vox.core import Transcriber
import logging

# The application using the library is responsible for configuring logging
logging.basicConfig(level=logging.INFO)

def handle_transcription(text):
    """This function will be called with each new transcription."""
    print(f"My application received: {text}")

# Configuration dictionary (see CLI arguments table for all options)
my_config = {
    "model": "base",
    "lang": "en",
    "silence_threshold": 150,
}

try:
    # Instantiate the transcriber with your config and callback
    transcriber = Transcriber(config=my_config, on_transcription_cb=handle_transcription)
    transcriber.run() # This is a blocking call
except KeyboardInterrupt:
    print("Stopping transcription.")
finally:
    if 'transcriber' in locals():
        transcriber.close()
```
---

## Dependencies and Manual Installation

Before installing via `pip`, some system-level setup is required.

1.  **Install PortAudio**: `pyaudio` requires the PortAudio C library.
    -   **Debian/Ubuntu**: `sudo apt-get install portaudio19-dev`
    -   **macOS (Homebrew)**: `brew install portaudio`

2.  **Note on `torch`**: Whisper depends on PyTorch (`torch`), which can be complex to install, especially for GPU support. If you encounter issues, we strongly recommend following the official instructions at [pytorch.org](https://pytorch.org/get-started/locally/).

---

## CLI Arguments Reference

| Argument                    | Default    | Description                                                                         |
| --------------------------- | ---------- | ----------------------------------------------------------------------------------- |
| `-m`, `--model`             | `base`     | Whisper model to use (`tiny`, `base`, `small`, `medium`, `large`).                           |
| `-g`, `--lang`              | `None`     | Language code (`en`, `de`). `None` for auto-detection.                              |
| `-t`, `--threshold`         | `1000`     | Pause duration in ms to trigger transcription.                                      |
| `-s`, `--silence-threshold` | `15`       | RMS audio level below which is considered silence.                                  |
| `-d`, `--device`            | `None`     | Index of the audio input device.                                                    |
| `-c`, `--chunk-size`        | `1024`     | Samples per audio chunk. Lower is more responsive.                                  |
| `--model-dir`               | `None`     | Directory to store/load Whisper models.                                             |
| `-p`, `--pre-buffer`        | `5`        | Chunks to keep before speech starts to avoid clipped words.                         |
| `-w`, `--warmup`            | `2`        | Consecutive loud chunks to start recording.                                         |
| `-r`, `--rms`               | `False`    | Show real-time audio volume (RMS).                                                  |
| `--channels`                | `1`        | Number of audio channels.                                                           |
| `--rate`                    | `16000`    | Audio sample rate in Hz.                                                            |
| `--output-handlers`         | `['stdout']`| A space-separated list of built-in handlers to use (e.g., `stdout`, `file`, `http_post`).|
| `--custom-handler`          | `None`     | Path to a Python script with a custom `OutputHandler`.                              |
| `-l`, `--list-devices`      | `False`    | List available audio input devices and exit.                                        |
| `--log-level`               | `info`     | Set the logging level. Use warning or lower to supress most of the logging. (`debug`, `info`, `warning`, `error`). |

---

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
---
For more details, advanced use cases, and to contribute to the project, please visit the **[Official GitHub Repository](https://github.com/bob-ros2/vox)**.