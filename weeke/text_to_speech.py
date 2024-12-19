#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def speak_text():
   
    rospy.init_node('speaker_node', anonymous=True)
    
    #setup TTS
    pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
    
    rospy.sleep(1)
    
    #publish string
    pub.publish(data="Hello, this is a test message")

if __name__ == '__main__':
    try:
        speak_text()
    except rospy.ROSInterruptException:
        pass
