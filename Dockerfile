# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Arguments for User and Group IDs, with a default of 1000
ARG UID=1000
ARG GID=1000

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    python3.10-venv \
    portaudio19-dev \
    ffmpeg \
    pulseaudio \
    libpulse0 \
    pulseaudio-utils \  
    && rm -rf /var/lib/apt/lists/*

# Create a group and user with the IDs passed during the build
RUN groupadd -g $GID appgroup && \
    useradd -u $UID -g $GID -m -s /bin/bash appuser && \
    usermod -a -G audio appuser

# prepare venv
RUN mkdir /opt/venv && \
    chown -R appuser:appgroup /opt/venv

# Set up the working directory
WORKDIR /app
RUN chown -R appuser:appgroup /app

# Switch to the new, non-root user
USER appuser

# Create and use a virtual environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python packages
RUN python -m pip install --upgrade pip && \
    pip install numpy pyaudio openai-whisper torch

# Install the project and its dependencies from pyproject.toml
COPY pyproject.toml .
COPY ./vox ./vox
RUN pip install .

# Configure PulseAudio to use host's socket
ENV PULSE_SERVER=unix:/run/pulse/native
VOLUME /run/pulse/native
VOLUME /models

# Set the default command to run when the container starts
CMD ["vox", "--model", "base", "--lang", "en", "--model-dir", "/models"]
