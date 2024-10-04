#!/usr/bin/python3
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # set initial angle
    wrist_angle = 0
    elbow_angle = 0
    
    # Controls the waving back and forth
    wave_direction = 1 

    while not rospy.is_shutdown():
        js = JointState()
        
        # header info
        js.header.stamp = rospy.get_rostime()
        js.header.frame_id = "Torso"
     
        # Define joint names
        js.name = [
            "HeadYaw", "HeadPitch", "RShoulderPitch", "RShoulderRoll",
            "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"
        ]

        # Define joint positions
        js.position = [
            0, 0,  # Head stays still
            math.radians(-45),  # RShoulderPitch (raise arm)
            math.radians(0),    # RShoulderRoll
            math.radians(0),    # RElbowYaw
            math.radians(elbow_angle),  # RElbowRoll (wave elbow)
            math.radians(wrist_angle),  # RWristYaw (wave wrist)
            0.5  # RHand (half open)
        ]

        rospy.loginfo(js)
        pub.publish(js)

        # Update angles for next iteration
        wrist_angle += 10 * wave_direction
        elbow_angle += 5 * wave_direction
        if abs(wrist_angle) >= 30 or abs(elbow_angle) >= 15:
            wave_direction *= -1  # Reverse the direction when it hits the limits

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    
    
    

# need arm joints and a 0 angle set for ones not going to move (torso, shoulder roll and pitch, then elbow and wrist)
# can get rid of others (or keep and set to 0)

# to change any angle in the "positions list" - tuple will need to be changed to list, then edit list, then back to tuple
        
        
