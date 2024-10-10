#!/usr/bin/python3
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import JointState

def nod_head(current_angle):
    #nod 'yes' by moving the head pitch
    nod_speed = 1  #head will nod 1 degree per second
    nod_limit = 15  #maximum angle the head will nod

    if abs(current_angle) >= nod_limit:
        return -current_angle  #head switches nod direction when max angle is reached
    else:
        return current_angle + nod_speed if current_angle >= 0 else current_angle - nod_speed

def shake_head(current_angle):
    #shake 'no' by moving the head yaw
    shake_speed = 2  #head will shake one degree per second
    shake_limit = 20  #maximum angle head will shake to one side

    if abs(current_angle) >= shake_limit:
        return -current_angle  # head switches direction when max angle is reached
    else:
        return current_angle + shake_speed if current_angle >= 0 else current_angle - shake_speed

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) 
    
    wrist_angle = 0.5
    elbow_angle = 0
    head_yaw = 0
    head_pitch = 0
    
    wave_direction = 1
    
    #switching between nodding and shaking
    movement_counter = 0
    
    while not rospy.is_shutdown():
        js = JointState()
        
        # header info
        js.header.stamp = rospy.get_rostime()
        js.header.frame_id = "Torso"
     
        # Define joint names
        js.name = [
            "HeadYaw", "HeadPitch",
            "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand",
            "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
            "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"
        ]
        
        if movement_counter < 25:  #nodding 'yes
            head_pitch = nod_head(head_pitch)
            head_yaw = 0
        elif movement_counter < 50:  # shaking 'no'
            head_yaw = shake_head(head_yaw)
            head_pitch = 0
        else:
            movement_counter = 0  #reset so sequence repeats
        
        movement_counter += 1
        
    
        js.position = [
            math.radians(head_yaw), math.radians(head_pitch),  #head
            0, 0, 0, 0, 0, 0,  #left arm
            math.radians(-45), 0, 0, math.radians(elbow_angle), math.radians(wrist_angle), 0.5,  #right arm
            0, 0, 0, 0, 0, 0,  #left leg
            0, 0, 0, 0, 0, 0   #right leg
        ]
        
        rospy.loginfo(js)
        pub.publish(js)
        
        
        wrist_angle += 10 * wave_direction
        elbow_angle += 10 * wave_direction
        
        if abs(wrist_angle) >= 30 or abs(elbow_angle) >= 15:
            wave_direction *= -1  #wave direction change
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



