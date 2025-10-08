import datetime
import argparse
import logging

class OutputHandler:
    def __init__(self, handler_args=None):
        
        parser = argparse.ArgumentParser(description='File Handler Arguments')

        parser.add_argument('--file-name', type=str, default='transcriptions.txt',
            help='The name of the file to save transcriptions to.')
        
        parser.add_argument('--file-no-timestamp', action='store_true',
            help='If set, timestamps will not be added within the output file.')

        self.args, _ = parser.parse_known_args(handler_args)

        self.file = open(self.args.file_name, "a")

    def send(self, text):

        if self.args.file_no_timestamp:
            self.file.write(f"{text}\n")
        else:
            timestamp = datetime.datetime.now().isoformat()
            self.file.write(f"{timestamp} | {text}\n")

        self.file.flush()

    def close(self):
        self.file.close()
