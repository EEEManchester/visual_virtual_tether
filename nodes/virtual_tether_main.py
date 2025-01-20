#!/usr/bin/env python

import math
import threading

import rospy
from geometry_msgs.msg import Twist,Point, Quaternion
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
import tf
from apriltag_ros.msg import AprilTagDetectionRawArray
from apriltag_ros.msg import AprilTagDetectionArray

from visual_virtual_tether.cfg import VirtualTetherConfig


GAIN_KP_DEFAULT = 0
GAIN_KD_DEFAULT = 1
YAW_OFFSET_DEFAULT = 0
LOCAL_DEAD_RECKONING_TIMEOUT_DEFAULT = 5
LOCAL_DEAD_RECKONING_FIXED_VELOCITY_DEFAULT = 0.1

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def scale_to_range(x, y, max_range):
    norm = math.sqrt(x**2 + y**2)
    k = max_range / norm
    return x*k, y*k


class VirtualTetherPparam:
    def __init__(self):
        self.config = None

    @property
    def gain_kp(self):
        return self.config["gain_kp"] if self.config else GAIN_KP_DEFAULT

    @property
    def gain_kd(self):
        return self.config["gain_kd"] if self.config else GAIN_KD_DEFAULT
    
    @property
    def tag_yaw_offset(self):
        return self.config["tag_yaw_offset"] if self.config else YAW_OFFSET_DEFAULT
    
    @property
    def local_dead_reckoning_timeout(self):
        return self.config["local_dead_reckoning_timeout"] if self.config else LOCAL_DEAD_RECKONING_TIMEOUT_DEFAULT
    
    @property
    def local_dead_reckoning_fixed_velocity(self):
        return self.config["local_dead_reckoning_fixed_velocity"] if self.config else LOCAL_DEAD_RECKONING_FIXED_VELOCITY_DEFAULT
    
    def set_reconfig(self, config):
        self.config = config
        rospy.loginfo("Virtual tether reconfiguration set")


class Virtual_tether:
    def __init__(self):
        rospy.init_node('Virtual_tether')

        self.lock = threading.RLock()
        self.target = Point(x=400, y=300)

        self.detection_ready = False
        self.detection = None
        self.detection_time = None
        self.dead_reckoning = False
        self.prev_error = 0
        self.vel_yaw = 0
        self.control_time = None
        self.prev_control_time = 0

        self.control_pub = rospy.Publisher('virtual_tether/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/tag_detections_raw', AprilTagDetectionRawArray, self.tag_detection_callback)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_raw_callback)

        self.param = VirtualTetherPparam()

    def init_reconfiguration(self):
        self.reconf_server = Server(VirtualTetherConfig, self.reconfigure_callback)
        self.reconf_client = Client('virtual_tether')
        self.reconf_client.update_configuration({
            "gain_kp": rospy.get_param('~gain_kp', GAIN_KP_DEFAULT),
            "gain_kd": rospy.get_param('~gain_kd', GAIN_KD_DEFAULT),
            "tag_yaw_offset": rospy.get_param('~tag_yaw_offset', YAW_OFFSET_DEFAULT),
            "local_dead_reckoning_timeout": rospy.get_param('~local_dead_reckoning_timeout', LOCAL_DEAD_RECKONING_TIMEOUT_DEFAULT),
            "local_dead_reckoning_fixed_velocity": rospy.get_param('~local_dead_reckoning_fixed_velocity', LOCAL_DEAD_RECKONING_FIXED_VELOCITY_DEFAULT),
        })

    def tag_raw_callback(self,msg):
        if msg.detections:
            q_yaw = msg.detections[0].pose.pose.pose.orientation
            quaternion_yaw = [q_yaw.x, q_yaw.y, q_yaw.z, q_yaw.w]
            euler_yaw = tf.transformations.euler_from_quaternion(quaternion_yaw)[2]
            euler_yaw = self.get_offset(euler_yaw)
            self.vel_yaw = euler_yaw / math.pi
        if abs(self.vel_yaw) > 0.13:
            self.vel_yaw = 0.13 if self.vel_yaw > 0 else -0.13

    def tag_detection_callback(self, msg):
        current_time = rospy.Time.now().to_sec()

        if msg.detections:
            with self.lock:
                self.detection = msg.detections[0]
                self.detection_time = current_time
                self.detection_ready = True
        else:
            with self.lock:
                self.detection_ready = False
            # rospy.loginfo_throttle(1, 'out of LoS!')

    def reconfigure_callback(self, config, level):
        rospy.loginfo("Virtual tether reconfiguration received.")
        self.param.set_reconfig(config)
        return config

    def get_offset(self, a):
        a += self.param.tag_yaw_offset
        if a > math.pi:
            a -= 2 * math.pi
        elif a < -math.pi:
            a += 2 * math.pi
        return a

    def vel_state(self, current, target, do_print=False):
        # current_time = rospy.Time.now().to_sec()
        error = (target - current)/(target)
        
        if not self.prev_control_time:
            d_term = 0
        else:
            time_diff = self.control_time - self.prev_control_time
            if time_diff == 0:
                d_term = 0
                rospy.logwarn("Time diff is 0, possibly caused by significant network delay?")
            d_term = (error - self.prev_error) / time_diff * self.param.gain_kd
        self.prev_error = error
        cmd = error * self.param.gain_kp + d_term
        return 6, cmd
        
    def get_detection(self):
        with self.lock:
            if self.detection_ready:
                self.dead_reckoning = False
                return self.detection
            if self.detection_time and self.control_time - self.detection_time < self.param.local_dead_reckoning_timeout:
                rospy.logdebug("Dead reckoning, detection is %f seconds old", self.control_time - self.detection_time)
                self.dead_reckoning = True
                return self.detection

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        cmd_vel = Twist()
        empty_twist = Twist()
        while not rospy.is_shutdown():
            self.prev_control_time = self.control_time
            self.control_time = rospy.Time.now().to_sec()
            with self.lock:
                detection = self.get_detection()
            if detection:
                x_state, v_x = self.vel_state(detection.centre.x, self.target.x, True)
                y_state, v_y = self.vel_state(detection.centre.y, self.target.y)
                cmd_vel.linear.x = 0.23*v_y
                cmd_vel.linear.y = 0.43*v_x
                cmd_vel.linear.z = 0
                cmd_vel.angular.x = y_state
                cmd_vel.angular.y = x_state
                cmd_vel.angular.z = -self.vel_yaw * 0.9
                if self.dead_reckoning:
                    if self.local_dead_reckoning_fixed_velocity:
                        cmd_vel.linear.x, cmd_vel.linear.y = scale_to_range(cmd_vel.linear.x, cmd_vel.linear.y, self.param.local_dead_reckoning_fixed_velocity)
                    cmd_vel.angular.z = 0
                self.control_pub.publish(cmd_vel)
            else:
                self.control_pub.publish(empty_twist)
            rate.sleep()

def main():
    node = Virtual_tether()
    node.run()
        
if __name__ == "__main__":
    main()