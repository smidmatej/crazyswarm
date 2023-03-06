"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import tf
import rospy

import geometry_msgs.msg
import math
import numpy as np
from PyKDL import Rotation, Vector, Frame

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
VICON_UPDATE_RATE = 100.0 # Hz
VICON_INTERVAL = rospy.Duration.from_sec(1.0 / VICON_UPDATE_RATE)

def main():

    #rospy.init_node('tf_velocity_calc')
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]


    motorPowerSet(cf, 10000*np.array([1, 1, 1, 1]))
    timeHelper.sleep(TAKEOFF_DURATION)
    motorPowerSet(cf, np.array([0, 0, 0, 0]))




def motorPowerSet(cf, motor_power : np.ndarray):
    rospy.logwarn(f"Setting motor power to {motor_power}")
    cf.setParam("motorPowerSet/enable", 1)
    cf.setParam("motorPowerSet/m1", int(motor_power[0]))
    cf.setParam("motorPowerSet/m2", int(motor_power[1]))
    cf.setParam("motorPowerSet/m3", int(motor_power[2]))
    cf.setParam("motorPowerSet/m4", int(motor_power[3]))

if __name__ == "__main__":
    main()
    rospy.spin()