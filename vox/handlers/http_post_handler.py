import requests
import argparse
import logging

class OutputHandler:
    def __init__(self, handler_args=None):
        # Each handler can have its own mini argument parser
        parser = argparse.ArgumentParser(description='HTTP Post Handler Arguments')
        parser.add_argument('--http-post-url', type=str, required=True,
                            help='The URL to which the transcription text will be POSTed.')
        parser.add_argument('--http-post-format', type=str, default='json', choices=['json', 'raw'],
                            help='The format of the POST request body.')

        # Parse only the arguments meant for this handler
        args, _ = parser.parse_known_args(handler_args)

        if not args.http_post_url:
            raise ValueError("--http-post-url is a required argument for the http_post handler.")

        self.url = args.http_post_url
        self.format = args.http_post_format
        logging.info(f"HTTP Post handler configured for URL: {self.url}")

    def send(self, text):
        try:
            if self.format == 'json':
                response = requests.post(self.url, json={'text': text}, timeout=3)
            else: # raw
                response = requests.post(self.url, data=text.encode('utf-8'), 
                                         headers={'Content-Type': 'text/plain'}, timeout=3)

            response.raise_for_status() # Raise an exception for bad status codes (4xx or 5xx)
        except requests.exceptions.RequestException as e:
            logging.error(f"HTTP Post handler failed to send data: {e}")

    def close(self):
        pass