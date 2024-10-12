#!/usr/bin/env python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

def get_hand_position(tfBuffer):
    try:
        trans = tfBuffer.lookup_transform('torso', 'l_gripper', rospy.Time())
        return trans.transform.translation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None

def calculate_head_angles(hand_position):
    yaw = math.atan2(hand_position.y, hand_position.x)
    pitch = math.atan2(-hand_position.z, math.sqrt(hand_position.x**2 + hand_position.y**2)) #this exagerates the gesture 
    return yaw, pitch

def gesture_toward_hand():
    rospy.init_node('nao_gesture_toward_hand')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        hand_position = get_hand_position(tfBuffer)
        if hand_position:
            yaw, pitch = calculate_head_angles(hand_position)
            
            #head gesture toward hand
            gesture = JointState()
            gesture.header.stamp = rospy.Time.now()
            gesture.name = ['HeadYaw', 'HeadPitch']
            gesture.position = [yaw, pitch]
            pub.publish(gesture)
            rospy.sleep(1)
            
            #head back to center
            gesture.position = [0, 0]
            pub.publish(gesture)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        gesture_toward_hand()
    except rospy.ROSInterruptException:
        pass

