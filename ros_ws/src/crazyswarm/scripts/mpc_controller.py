"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import tf
import rospy

import geometry_msgs.msg
from crazyswarm.msg import FullState13

import math
import numpy as np
from PyKDL import Rotation, Vector, Frame

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
VICON_UPDATE_RATE = 100.0 # Hz
VICON_INTERVAL = rospy.Duration.from_sec(1.0 / VICON_UPDATE_RATE)


class MPCController:
    def __init__(self):
        
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        #rospy.init_node('mpc_controller')

        self.cf_name = "cf4"

        self.FullStateSubsrciber = rospy.Subscriber("/" + self.cf_name + "/full_state", FullState13, self.FullStateCallback)


    def FullStateCallback(self, msg):

        x_world = self.FullStateMsg_to_np(msg)
        #rospy.logwarn(f"msg time = {msg.header.stamp.secs}.{msg.header.stamp.nsecs}")
        #rospy.logwarn(x_world)
        #rospy.loginfo(f"pos = {x_world[:3]}")
        #rospy.loginfo(f"vel = {x_world[7:10]}")
        #rospy.loginfo(f"orientation = {x_world[3:7]}")
        #rospy.loginfo(f"angular vel = {x_world[10:]}")


        z_ref = 1.0
        k = 10000.0
        u_z = k*(z_ref - x_world[2])
        #rospy.loginfo(f"z = {x_world[2]}")
        #rospy.loginfo(f"u_z = {u_z}")
        motor_power = u_z*np.array([1, 1, 1, 1])
        motorPowerSet(self.cf, motor_power)
        

    def FullStateMsg_to_np(self, msg):
        # World frame state
        state_world = np.array([
                        msg.pose.position.x, 
                        msg.pose.position.y, 
                        msg.pose.position.z, 
                        msg.pose.orientation.w, 
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z, 
                        msg.twist.linear.x, 
                        msg.twist.linear.y, 
                        msg.twist.linear.z, 
                        msg.twist.angular.x, 
                        msg.twist.angular.y, 
                        msg.twist.angular.z])
        return state_world


def motorPowerSet(cf, motor_power : np.ndarray):
    #rospy.logwarn(f"Setting motor power to {motor_power}")
    cf.setParam("motorPowerSet/enable", 1)
    cf.setParam("motorPowerSet/m1", int(motor_power[0]))
    cf.setParam("motorPowerSet/m2", int(motor_power[1]))
    cf.setParam("motorPowerSet/m3", int(motor_power[2]))
    cf.setParam("motorPowerSet/m4", int(motor_power[3]))

if __name__ == "__main__":
    MPCController()
    rospy.spin()