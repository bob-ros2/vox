# Vox-Real: Real-time Whisper Transcription Tool & Library

**Vox-Real** is a powerful, real-time audio transcription tool and Python library that uses OpenAI's Whisper model. It continuously listens to a microphone, detects speech, and transcribes it upon pauses, making it ideal for a wide range of voice-activated applications.

This package can be used in two primary ways:
1.  **As a standalone command-line tool** (`vox`).
2.  **As a Python library** to integrate real-time transcription into your own applications.

> **Note**: This is the README for the PyPI package. For more advanced features, including **Docker support and contribution guidelines**, please visit the full project repository on GitHub:
> - [https://github.com/bob-ros2/vox](https://github.com/bob-ros2/vox)


---

## Installation

First, ensure you have the necessary system-level dependencies (see the "Dependencies" section below). Then, you can install the package directly from PyPI. It is highly recommended to use a virtual environment.

```bash
pip install vox-real
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
-   **Pluggable Output System**: Send transcribed text to the console, files, HTTP webhook, or ROS2 topics.

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
    "silence_threshold": 30,
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
| `-s`, `--silence-threshold` | `20`       | RMS audio level below which is considered silence.                                  |
| `-d`, `--device`            | `None`     | Index of the audio input device.                                                    |
| `-c`, `--chunk-size`        | `1024`     | Samples per audio chunk. Lower is more responsive.                                  |
| `--model-dir`               | `None`     | Directory to store/load Whisper models.                                             |
| `-p`, `--pre-buffer`        | `5`        | Chunks to keep before speech starts to avoid clipped words.                         |
| `-w`, `--warmup`            | `2`        | Consecutive loud chunks to start recording.                                         |
| `-r`, `--rms`               | `False`    | Show real-time audio volume (RMS).                                                  |
| `--channels`                | `1`        | Number of audio channels.                                                           |
| `--rate`                    | `16000`    | Audio sample rate in Hz.                                                            |
| `--whisper-device`          | `None`     | Device to use for Whisper inference (`cpu`, `cuda`, `cuda:0`). `None` for auto-detect.|
| `--output-handlers`         | `['stdout']`| A space-separated list of built-in handlers to use (e.g., `stdout`, `file`, `http_post`, `ros2`).|
| `--custom-handler`          | `None`     | Path to a Python script with a custom `OutputHandler`.                              |
| `-l`, `--list-devices`      | `False`    | List available audio input devices and exit.                                        |
| `--log-level`               | `info`     | Set the logging level. Use warning or lower to supress most of the logging. (`debug`, `info`, `warning`, `error`). |
| `--initial-prompt`          | `None`     | Optional text to provide context or specify output style.                           |
| `--temperature`             | `0.0`      | Sampling temperature. 0.0 is deterministic. Higher values are more random.          |
| `--beam-size`               | `None`     | Number of beams in beam search. None uses Whisper default.                          |
| `--patience`                | `None`     | Beam search patience factor.                                                        |
| `--length-penalty`          | `None`     | Beam search length penalty.                                                         |
| `--no-condition-on-previous-text` | `False` | Disable conditioning on previous text (prevents loops).                             |
| `--no-fp16`                 | `False`    | Disable FP16 computation (force FP32).                                              |

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

#### `ros2` handler
ROS2 (Robot Operating System 2) is an open-source software framework for robots.
| Argument | Default | Description |
|---|---|---|
| `--ros2-topic` | `transcription` | The ROS2 topic to publish transcriptions to. |

---

## Robust Configuration Examples

Here are some examples of how to configure `vox` for different scenarios, building on the basic command:
`vox --model medium --lang de --rms --output-handlers stdout ros2 --ros2-topic /you/llm_prompt`

### 1. Reducing Hallucinations (Repetitions)
If the model gets stuck repeating the same phrase or hallucinating silence, try increasing the temperature slightly and disabling conditioning on previous text.
```bash
vox ... --temperature 0.2 --no-condition-on-previous-text
```

### 2. Improving Context and Style
Use `--initial-prompt` to tell the model about the context (e.g., technical terms, specific spelling) or the desired style.
```bash
vox ... --initial-prompt "Dies ist ein Transkript über Robotik und ROS2."
```

### 3. Maximizing Accuracy (Slower)
Increase the `beam-size` to explore more decoding paths. This will increase latency but may improve accuracy for complex sentences.
```bash
vox ... --beam-size 10
```

### 4. Low-Resource / Compatibility Mode
If you encounter errors related to FP16 on older hardware, force FP32.
```bash
vox ... --no-fp16
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
For more details, advanced use cases, and to contribute to the project, please visit the **[Official GitHub Repository](https://github.com/bob-ros2/vox)**.