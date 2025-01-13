#!/usr/bin/env python

import threading

import rospy
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import Twist

from visual_virtual_tether.cfg import MallardMixerConfig

class mallard_to_twist_param:
    def __init__(self):
        self.config = None

    @property
    def gain_x(self):
        return self.config.gain_x if self.config else 1

    @property
    def gain_y(self):
        return self.config.gain_y if self.config else 1

    @property
    def gain_yaw(self):
        return self.config.gain_yaw if self.config else 1

    @property
    def joy_gain_x(self):
        return self.config.joy_gain_x if self.config else 1

    @property
    def joy_gain_y(self):
        return self.config.joy_gain_y if self.config else 1

    @property
    def joy_gain_yaw(self):
        return self.config.joy_gain_yaw if self.config else 1

    @property
    def vt_gain_x(self):
        return self.config.vt_gain_x if self.config else 1

    @property
    def vt_gain_y(self):
        return self.config.vt_gain_y if self.config else 1

    @property
    def vt_gain_yaw(self):
        return self.config.vt_gain_yaw if self.config else 1

    def set_reconfig(self, config):
        self.config = config
        rospy.loginfo("Reconfiguration set.")

class mallard_to_twist:
    def __init__(self):
        rospy.init_node('virtuaL_tether_mallard_mixer')
        self.data_lock = threading.RLock()
        self.roll1 = 0.0
        self.pitch1 = 0.0
        self.yaw1 = 0.0
        self.thrust1 = 0.0
        self.vertical_thrust1 = 0.0
        self.lateral_thrust1 = 0.0
        self.roll2 = 0.0
        self.pitch2 = 0.0
        self.yaw2 = 0.0
        self.thrust2 = 0.0
        self.vertical_thrust2 = 0.0
        self.lateral_thrust2 = 0.0
        self.x_danger_on = 1
        self.y_danger_on = 1
        self.x_safe_on = 1
        self.y_safe_on = 1

        self.initialise_reconf_server()

        self.control_pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('virtual_tether/cmd_vel', Twist, self.virtual_tether_cmd_vel_callback)
        rospy.Subscriber('/mallard/teleop/cmd_vel', Twist, self.teleop_cmd_vel_callback)

    def initialise_reconf_server(self):
        self.params = mallard_to_twist_param()
        self.reconf_server = Server(MallardMixerConfig, self.reconfigure_callback)
        self.reconf_client = Client('virtuaL_tether_mallard_mixer')
        self.reconf_client.update_configuration({
            "gain_x": rospy.get_param('virtuaL_tether_mallard_mixer/overall_gain/x', 1),
            "gain_y": rospy.get_param('virtuaL_tether_mallard_mixer/overall_gain/y', 0.8),
            "gain_yaw": rospy.get_param('virtuaL_tether_mallard_mixer/overall_gain/yaw', 0.5),
            "joy_gain_x": rospy.get_param('virtuaL_tether_mallard_mixer/joy_gain/x', 1),
            "joy_gain_y": rospy.get_param('virtuaL_tether_mallard_mixer/joy_gain/y', 0.8),
            "joy_gain_yaw": rospy.get_param('virtuaL_tether_mallard_mixer/joy_gain/yaw', 0.5),
            "vt_gain_x": rospy.get_param('virtuaL_tether_mallard_mixer/virtual_tether_gain/x', 1),
            "vt_gain_y": rospy.get_param('virtuaL_tether_mallard_mixer/virtual_tether_gain/y', 0.8),
            "vt_gain_yaw": rospy.get_param('virtuaL_tether_mallard_mixer/virtual_tether_gain/yaw', 0.5)
        })

    def reconfigure_callback(self, config, level):
        rospy.loginfo("Reconfiguration received.")
        self.params.set_reconfig(config)
        return config

    def virtual_tether_cmd_vel_callback(self, msg):
        with self.data_lock:
            if msg.angular.x == 5:
                # rospy.loginfo("x safe")
                self.thrust2 = 2.5*msg.linear.x
                self.x_danger_on = 1
                self.x_safe_on = 1
            if msg.angular.x == 6:
                # rospy.loginfo("x elastic")
                self.thrust2 = 2.5*msg.linear.x
                self.x_danger_on = 1
                self.x_safe_on = 1
            if msg.angular.x == 9:
                # rospy.loginfo("x danger")
                self.x_danger_on = 0
                self.thrust2 = 2.5*msg.linear.x 
                self.x_safe_on = 1  

            if msg.angular.y == 5:
                # rospy.loginfo("y safe")
                self.lateral_thrust2 = 3*msg.linear.y
                self.y_danger_on = 1
                self.y_safe_on = 1

            if msg.angular.y == 6:
                # rospy.loginfo("y elastic")
                self.lateral_thrust2 = 3*msg.linear.y
                self.y_danger_on = 1
                self.y_safe_on = 1

            if msg.angular.y == 9:
                # rospy.loginfo("y danger")
                self.y_danger_on = 0
                self.y_safe_on = 1
                self.thrust2 = 3*msg.linear.y   
            
            self.thrust2 *= self.params.vt_gain_x
            self.lateral_thrust2 *= self.params.vt_gain_y
            self.vertical_thrust2 = 0
            self.pitch2 = 0
            self.roll2 = 0
            self.yaw2 = self.params.vt_gain_yaw * msg.angular.z

    def teleop_cmd_vel_callback(self, msg):
        with self.data_lock:
            self.thrust1 = self.params.joy_gain_x * msg.linear.x
            self.lateral_thrust1 = self.params.joy_gain_y * msg.linear.y
            self.vertical_thrust1 = 0
            self.pitch1 = 0
            self.roll1 = 0
            self.yaw1 = self.params.joy_gain_yaw * msg.angular.z

    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            to_twist = Twist()
            
            #experiemts
            # to_twist.linear.x = 0.5*(self.thrust1 * self.x_safe_on * self.x_danger_on + self.thrust2)
            # to_twist.linear.y = 0.5*(self.lateral_thrust1 * self.y_safe_on * self.y_danger_on + self.lateral_thrust2)
            
            #sim
            to_twist.linear.x = self.params.gain_x * (self.thrust1 * self.x_safe_on * self.x_danger_on + self.thrust2)
            to_twist.linear.y = self.params.gain_y * (self.lateral_thrust1 * self.y_safe_on * self.y_danger_on + self.lateral_thrust2)

            # #sim for vvt slam problem 0813
            # to_twist.linear.x = (self.thrust1 + 0*self.thrust2)
            # to_twist.linear.y = 0.8*(self.lateral_thrust1 + 0*self.lateral_thrust2)


            to_twist.linear.z = self.vertical_thrust1
            to_twist.angular.x = self.pitch1
            to_twist.angular.y = self.roll1
            to_twist.angular.z = self.params.gain_yaw * (self.yaw1+self.yaw2)
            self.control_pub.publish(to_twist)

            rate.sleep()
    
def main():
    node = mallard_to_twist()
    node.run()
        
if __name__ == "__main__":
    main()