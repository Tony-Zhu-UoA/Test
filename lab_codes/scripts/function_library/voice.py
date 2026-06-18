#!/usr/bin/env python
# \author  Zeqiang Zhu(Tony) zzhu488@aucklanduni.ac.nz
# \date    2025-04
import threading
import signal
import time
import pyaudio
from google.cloud import speech
from google.cloud import texttospeech
from google.oauth2 import service_account
import queue

"""
It is used to speech to text and text to speech. You should get your own API credential in Google Cloud and add the right path when you use it. 
Don’t change them!!!

Example usage:
from function_library.voice import VoiceAssistant
credentials_path = "/home/352lab/hrc_real/hrc_ws/hrc_interface/scripts/psyched-thunder-454702-f7-a5bf7cfae33f.json" # Path to your Google Cloud API credentials
voice = VoiceAssistant(credentials_path)

# Example: Convert text to speech
voice.text_to_speech("Hello, how can I assist you?")

# Example: Start speech-to-text recognition
# Create a thread to run speech_to_text_recognize()
recognition_thread = threading.Thread(target=assistant.speech_to_text_recognize)
recognition_thread.daemon = True 
recognition_thread.start()

recognized_text = voice.transcript_queue.get(timeout=5)
print(f"Main function received: {recognized_text}")
if "who are you" in recognized_text.lower():
    voice.text_to_speech("I am Tina.")
"""

class VoiceAssistant:
    def __init__(self, credentials_path):
        # Google Cloud credentials
        self.credentials = service_account.Credentials.from_service_account_file(credentials_path)

        # Initialize the queue
        self.transcript_queue = queue.Queue()

        self.first_time_prompt = True
        
        # Audio settings
        self.RATE = 16000
        self.CHUNK = int(self.RATE / 10)

        # Speech-to-Text client
        self.client = speech.SpeechClient(credentials=self.credentials)
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.RATE,
            language_code="en-US",
        )
        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config,
            interim_results=True
        )

        # Text-to-Speech client
        self.tts_client = texttospeech.TextToSpeechClient(credentials=self.credentials)

    def get_transcript(self, timeout=5):
        """
        Safely get the recognized text from the transcript queue.
        """
        try:
            return self.transcript_queue.get(timeout=timeout)
        except queue.Empty:
            print("No transcript available within the timeout period.")
            return None
        
    def text_to_speech(self, text):
        """
        Convert text to speech and play it.
        """
        text = "I. " + text
        input_text = texttospeech.SynthesisInput(text=text)
        voice = texttospeech.VoiceSelectionParams(
            language_code="en-US",
            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
            name="en-US-Neural2-F"
        )
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.RATE,
            speaking_rate=1.1,
            pitch=4.0
        )
        response = self.tts_client.synthesize_speech(
            input=input_text,
            voice=voice,
            audio_config=audio_config
        )

        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=self.RATE,
                        output=True)
        stream.write(response.audio_content)
        stream.stop_stream()
        stream.close()
        p.terminate()

    def microphone_stream(self):
        """
        Generator that yields audio chunks from the microphone.
        """
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK,
        )
        try:
            while True:
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                yield data
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    def speech_to_text_recognize(self):
        """
        Perform real-time speech-to-text recognition and store recognized text in a queue.
        """
        retry_count = 0
        max_retries = 5
        while True:
            try:
                requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in self.microphone_stream()
                )
                print("Please talk now.")
                responses = self.client.streaming_recognize(config=self.streaming_config, requests=requests)
                if self.first_time_prompt:
                    self.text_to_speech("Oh! Hey there! What was your name again?")
                    self.first_time_prompt = False
                for response in responses:
                    for result in response.results:
                        if result.is_final:
                            recognized_text = result.alternatives[0].transcript
                            print("You said: ", recognized_text)

                            # if "who are you" in recognized_text.lower():
                            #     voice.text_to_speech("I am Tina.")
                            # Store the recognized text in the queue
                            self.transcript_queue.put(recognized_text)
                retry_count = 0  # Reset retry count on success
            except Exception as e:
                retry_count += 1
                print(f"Restarting speech recognition (attempt {retry_count}/{max_retries}):", e)
                if retry_count >= max_retries:
                    print("Max retries reached. Exiting...")
                    break
                time.sleep(2 ** retry_count)  # Exponential backoff





