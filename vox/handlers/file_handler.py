import datetime

class OutputHandler:
    def __init__(self, filename="transcriptions.txt", **kwargs):
        self.file = open(filename, "a")

    def send(self, text):
        timestamp = datetime.datetime.now().isoformat()
        self.file.write(f"{timestamp} | {text}\n")
        self.file.flush()

    def close(self):
        self.file.close()
