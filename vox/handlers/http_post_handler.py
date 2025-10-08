import requests
import argparse
import logging

class OutputHandler:
    def __init__(self, handler_args=None):

        parser = argparse.ArgumentParser(description='HTTP Post Handler Arguments')
        
        parser.add_argument('--http-post-url', type=str, required=True,
            help='The URL to which the transcription text will be POSTed.')
        
        parser.add_argument('--http-post-format', type=str, default='json', choices=['json', 'raw'],
            help='The format of the POST request body.')

        parser.add_argument('--http-post-header', type=str, default=None,
            help='A JSON string representing key/value pairs for request headers.')

        # Parse only the arguments meant for this handler
        args, _ = parser.parse_known_args(handler_args)

        if not args.http_post_url:
            raise ValueError("--http-post-url is a required argument for the http_post handler.")

        self.url = args.http_post_url
        self.format = args.http_post_format

        self.headers = {}
        if args.http_post_header:
            try:
                self.headers = json.loads(args.http_post_header)
                if not isinstance(self.headers, dict):
                    raise ValueError("Header JSON must be a dictionary (object).")
                logging.info(f"Using custom HTTP headers: {self.headers}")
            except (json.JSONDecodeError, ValueError) as e:
                logging.error(f"Failed to parse --http-post-header JSON: {e}")

        logging.info(f"HTTP Post handler configured for URL: {self.url}")

    def send(self, text):
        try:
            if self.format == 'json':
                response = requests.post(
                    self.url, json={'text': text}, timeout=3, headers=self.headers)
            else: # raw
                # Start with custom headers and add default Content-Type if not provided
                request_headers = self.headers.copy()
                request_headers.setdefault('Content-Type', 'text/plain')

                response = requests.post(self.url, data=text.encode('utf-8'), 
                    headers=request_headers, timeout=3)

            response.raise_for_status() # Raise an exception for bad status codes (4xx or 5xx)
        except requests.exceptions.RequestException as e:
            logging.error(f"HTTP Post handler failed to send data: {e}")

    def close(self):
        pass