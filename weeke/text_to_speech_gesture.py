#!/usr/bin/env python3
import rospy
import pyttsx3

def wave():
    rospy.loginfo("Waving...")

def nod():
    rospy.loginfo("Nodding (Yes)...")

def shake():
    rospy.loginfo("Shaking head (No)...")

def text_to_speech_with_gesture():
    rospy.init_node("text_to_speech_gesture_node", anonymous=True)
    rospy.loginfo("Starting Text-to-Speech with Gesture Node")

    tts_engine = pyttsx3.init()
    speech = "hello this is a test yes or no"

    #split into words and trigger appropriate gestures
    rospy.loginfo(f"Speaking: {speech}")
    for word in speech.split():
        if word.lower() == "hello":
            wave()
        elif word.lower() == "yes":
            nod()
        elif word.lower() == "no":
            shake()
        tts_engine.say(word)
        tts_engine.runAndWait()

if __name__ == "__main__":
    try:
        text_to_speech_with_gesture()
    except rospy.ROSInterruptException:
        pass
