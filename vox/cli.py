import argparse
import importlib
import logging
from .core import Transcriber

def main():
    logging.basicConfig(
        level=logging.INFO, 
        format='%(asctime)s %(levelname)s %(message)s')
    
    # All your argparse code goes here, exactly as before
    parser = argparse.ArgumentParser(description='Real-time audio transcription program.')

    parser.add_argument('-t', '--threshold', type=int, default=1000, 
        help='Time in milliseconds a pause must last to end the recording (default: 1000).')

    parser.add_argument('-d', '--device', type=int, default=None,
        help='Index of the audio input device to use (e.g., -d 1). Default is the system\'s default input device.')

    parser.add_argument('-m', '--model', type=str, default='base',
        help='Name of the Whisper model to use (e.g., tiny, base, small). Default: base.')

    parser.add_argument('--model-dir', type=str, default=None,
        help='Directory to store/load Whisper models from.')

    parser.add_argument('-c', '--chunk-size', type=int, default=1024,
        help='Number of samples per audio chunk (default: 1024).')

    parser.add_argument('-s', '--silence-threshold', type=int, default=15,
        help='RMS threshold below which an audio chunk is considered silent (default: 15).')

    parser.add_argument('-l', '--lang', type=str, default=None,
        help='Language to transcribe in (e.g., "en", "de", "fr" ...). Default is auto-detection.')

    parser.add_argument('-p', '--pre-buffer', type=int, default=5,
        help='Number of audio chunks to keep in a buffer before speech is detected (default: 5).')

    parser.add_argument('-w', '--warmup', type=int, default=2,
        help='Number of consecutive loud chunks required to start recording (default: 2).')

    parser.add_argument('-r', '--rms', action='store_true',
        help='Show realtime volume RMS.')

    parser.add_argument('--channels', type=int, default=1,
        help='Number of audio channels (default: 1).')

    parser.add_argument('--rate', type=int, default=16000,
        help='Audio sample rate in Hz (default: 16000).')

    parser.add_argument('--output-handlers', type=str, nargs='+', default=['stdout'],
        help='A list of output handlers to use (e.g., --output-handlers stdout file).')

    parser.add_argument('--custom-handler', type=str, default=None,
        help='Path to a Python script file containing a custom OutputHandler class.')

    args, handler_args = parser.parse_known_args()

    # --- Load the Output Handlers ---
    output_handlers = []
    for handler_name in args.output_handlers:
        try:
            # The import path is relative to the vox package
            handler_module = importlib.import_module(
                f"vox.handlers.{handler_name}_handler")
            OutputHandler = getattr(handler_module, "OutputHandler")
            # Pass the remaining arguments to the handler's constructor
            output_handlers.append(OutputHandler(handler_args=handler_args))
            logging.info(f"Successfully loaded output handler: {handler_name}")
        except ImportError:
            logging.error(f"Could not find or import the output handler: '{handler_name}'.")
            logging.error(f"Please ensure a corresponding 'handlers/{handler_name}_handler.py' file exists.")
            exit(1)

    # --- Load Custom Handler from Script ---
    if args.custom_handler:
        try:
            logging.info(
                f"Attempting to load custom handler from: {args.custom_handler}")
            spec = importlib.util.spec_from_file_location(
                "custom_handler", args.custom_handler
            )
            custom_handler_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(custom_handler_module)
            
            CustomOutputHandler = getattr(custom_handler_module, "OutputHandler")
            output_handlers.append(CustomOutputHandler(handler_args=handler_args))
            
            logging.info(
                f"Successfully loaded custom handler from script.")
        except FileNotFoundError:
            logging.error(
                f"Custom handler script not found at: {args.custom_handler}")
            exit(1)
        except AttributeError:
            logging.error(
                f"The script at '{args.custom_handler}' does not contain an 'OutputHandler' class.")
            exit(1)
        except Exception as e:
            logging.error(
                f"Failed to load custom handler from script: {e}")
            exit(1)

    # --- Callback function that sends text to all handlers ---
    def forward_to_handlers(text):
        for handler in output_handlers:
            handler.send(text)

    # --- Use your library ---
    try:
        # Convert argparse namespace to a dictionary for the library
        config = vars(args)
        
        # Instantiate and run the transcriber
        transcriber = Transcriber(
            config=config, 
            on_transcription_callback=forward_to_handlers)
        transcriber.run()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Clean up handlers and the transcriber
        for handler in output_handlers:
            if hasattr(handler, 'close'):
                handler.close()
        if 'transcriber' in locals() and transcriber:
            transcriber.close()

if __name__ == '__main__':
    main()