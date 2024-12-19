#!/usr/bin/env python3
import rospy
import pyttsx3
from vosk import Model, KaldiRecognizer
import pyaudio
import json

def repeat_speech():
    rospy.init_node("speech_repeater", anonymous=True)
    rospy.loginfo("Starting Speech-to-Speech Node")

    #setup vosk
    model_path = "hri2024/src/hri_2024/weeke"
    model = Model(model_path)
    recognizer = KaldiRecognizer(model, 16000)
    audio = pyaudio.PyAudio()
    stream = audio.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
    stream.start_stream()

    #setup TTS
    tts_engine = pyttsx3.init()

    rospy.loginfo("Listening for speech...")
    while not rospy.is_shutdown():
        data = stream.read(4000, exception_on_overflow=False)
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get("text", "")
            if text:
                rospy.loginfo(f"Recognized: {text}")
                tts_engine.say(text)
                tts_engine.runAndWait()

if __name__ == "__main__":
    try:
        repeat_speech()
    except rospy.ROSInterruptException:
        pass
