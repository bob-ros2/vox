class OutputHandler:
    """
    A simple handler that prints the transcribed text to standard output.
    """
    def __init__(self, **kwargs):
        # This handler doesn't need any special initialization.
        pass

    def send(self, text):
        """
        Prints the text to stdout. flush=True is important for real-time piping.
        """
        print(text, flush=True)

    def close(self):
        # No resources to clean up.
        pass
