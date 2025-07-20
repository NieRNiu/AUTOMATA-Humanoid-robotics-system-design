# Humanoid Robot Voice Interaction & Navigation System

This project demonstrates a humanoid robot system with real-time voice interaction and navigation, powered by LLM-based dialogue reasoning.

## Features

- Real-time ASR (Automatic Speech Recognition)
- LLM-based dialogue and intent understanding
- TTS (Text-to-Speech) synthesis and playback
- Voice-command-based navigation in a ROS-Gazebo simulation

## System Requirements

- Ubuntu 20.04
- ROS Noetic
- Gazebo
- NVIDIA GPU with at least 6GB VRAM
- Conda (for environment setup)

## Setup

1. Create the environment:
   ```bash
   conda env create -f environment.yml
   ```

2. Fill in your authentication credentials in `api.py`:
   ```python
   ARK_API_KEY = "your-api-key"
   LLM_MODEL = "doubao-1-5-pro-32k-250115"
   APP_ID = "your-app-id"
   APP_TOKEN = "your-app-token"
   ```

## Components

### 1. ASR - Real-time Speech Recognition

Captures and transcribes audio input:
```bash
python asr.py
```

### 2. LLM - Large Language Model Dialogue

Processes dialogue input and determines intent:
```bash
python llm.py
```

### 3. TTS - Text-to-Speech

Synthesizes and plays back responses:
```bash
python tts.py
```

### 4. Integrated Demo

Supports full interaction flow and navigation command publishing. You can configure known locations in `prompt.py`, e.g.:
```python
"Live Room": (30, 20)
```

Then run:
```bash
python demo.py
```

When a user says "Go to the Live Room", the system recognizes the intent and publishes a goal via:
```python
Nav(x, y)
```

## Deliverables

- Source code
- Documentation
- Running demo in Gazebo simulation
