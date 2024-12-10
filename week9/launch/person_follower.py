#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from people_msgs.msg import PositionMeasurementArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import math
import tf

class LegFollower:
    def __init__(self):
        rospy.init_node('leg_follower', anonymous=True)
        
        #publishers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_pub = rospy.Publisher('/leg_follower_debug', Marker, queue_size=10)
        
        #subscribers
        self.legs_subscriber = rospy.Subscriber('/people_tracked', PositionMeasurementArray, self.legs_callback)
        self.scan_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        
        #TF listener
        self.tf_listener = tf.TransformListener()
        
        #movement parameters
        self.linear_speed = 0.3  #pretty low to keep it safe
        self.angular_speed = 0.5
        self.min_safe_distance = 0.5
        self.target_distance = 1.0
        
        #state
        self.person_detected = False
        self.current_person_pos = None
        self.laser_data = None
        
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        
        rospy.loginfo("Leg follower initialized. Starting...")

    def scan_callback(self, scan_data):
        """Process laser scan data"""
        self.laser_data = scan_data
        rospy.loginfo_throttle(5, f"Receiving laser scan data. Readings: {len(scan_data.ranges)}")
        
        #displaying the scan stats
        valid_readings = [r for r in scan_data.ranges if not math.isinf(r) and not math.isnan(r)]
        if valid_readings:
            rospy.loginfo_throttle(5, f"Scan stats - Min: {min(valid_readings):.2f}, Max: {max(valid_readings):.2f}")

    def legs_callback(self, msg):
        """Process leg detection data"""
        if not msg.people:
            self.person_detected = False
            self.current_person_pos = None
            rospy.loginfo_throttle(3, "No people detected")
            return
            
        #find the closest person
        closest_person = None
        min_distance = float('inf')
        
        for person in msg.people:
            #calculate distance from robot to person
            distance = math.sqrt(person.pos.x**2 + person.pos.y**2)
            if distance < min_distance:
                min_distance = distance
                closest_person = person
        
        if closest_person:
            self.current_person_pos = closest_person.pos
            self.person_detected = True
            self.publish_debug_marker(closest_person.pos.x, closest_person.pos.y)
            rospy.loginfo(f"Following person at x:{closest_person.pos.x:.2f}, y:{closest_person.pos.y:.2f}")

    def publish_debug_marker(self, x, y):
        """Publish visualization marker for detected person"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.debug_pub.publish(marker)

    def is_path_clear(self):
        """Check if path ahead is clear of obstacles"""
        if not self.laser_data:
            return False
            
        #scan the front arc of the robot (+-30 degrees)
        center_idx = len(self.laser_data.ranges) // 2
        arc_size = len(self.laser_data.ranges) // 6
        
        for i in range(center_idx - arc_size, center_idx + arc_size):
            if i >= 0 and i < len(self.laser_data.ranges):
                distance = self.laser_data.ranges[i]
                if not math.isinf(distance) and not math.isnan(distance):
                    if distance < self.min_safe_distance:
                        return False
        return True

    def move_to_person(self):
        """Calculate and execute movement towards person"""
        if not self.person_detected or not self.current_person_pos:
            self.stop_moving()
            return
            
        #calculate the angle and distance to person
        angle = math.atan2(self.current_person_pos.y, self.current_person_pos.x)
        distance = math.sqrt(self.current_person_pos.x**2 + self.current_person_pos.y**2)
        
        #ddjust the angular velocity to make robot face the person
        self.vel_msg.angular.z = self.angular_speed * angle
        
        #if we're pointing roughly at the person and path is clear, move forward
        if abs(angle) < 0.9 and self.is_path_clear(): 
            if distance > self.target_distance:
                #adjust sped as robot gets closer to person
                self.vel_msg.linear.x = min(self.linear_speed, 
                                          0.3 * (distance - self.target_distance))
            else:
                self.vel_msg.linear.x = 0
        else:
            #stop and turn
            self.vel_msg.linear.x = 0
            
        #publish the velocity 
        self.velocity_publisher.publish(self.vel_msg)
        
    def stop_moving(self):
        """Stop all robot movement"""
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting to follow legs...")
        
        while not rospy.is_shutdown():
            try:
                #check if TF is available
                if self.tf_listener.canTransform("/base_link", "/odom", rospy.Time(0)):
                    
                    #move if person is detected
                    if self.person_detected:
                        self.move_to_person()
                    else:
                        self.stop_moving()
                else:
                    rospy.logwarn_throttle(5, "No transform available between base_link and odom")
                    self.stop_moving()
                    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}")
                self.stop_moving()
            
            self.rate.sleep()

def main():
    try:
        follower = LegFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()