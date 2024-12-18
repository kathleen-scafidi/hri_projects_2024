#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def speak_text():
    # Initialize the ROS node
    rospy.init_node('speaker_node', anonymous=True)
    
    # Create a publisher to the TTS topic
    pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
    
    # Wait a moment to ensure the publisher is ready
    rospy.sleep(1)
    
    # Publish a string to speak
    pub.publish(data="Hello, this is a test message")

if __name__ == '__main__':
    try:
        speak_text()
    except rospy.ROSInterruptException:
        pass
