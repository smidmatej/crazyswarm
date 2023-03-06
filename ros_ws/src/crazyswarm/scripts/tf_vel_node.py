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

class FullState13PublisherNode():

    def __init__(self):


        cf_name = "cf4"
        rospy.init_node(f'tf_velocity_{cf_name}')

        self.FullStatePublisher = rospy.Publisher(cf_name + "/full_state", FullState13, queue_size=1)
        self.TwistStampedPublisher = rospy.Publisher(cf_name + "/twist_stamped", geometry_msgs.msg.WrenchStamped, queue_size=1)
        self.FullStateMsg = FullState13()
        self.FullStateMsg.header.seq = 0
        self.FullStateMsg.header.frame_id = "/world"

        # TODO: I cant see the twist in rviz. I need it to validate my velocity
        self.TwistStampedMsg = geometry_msgs.msg.WrenchStamped()
        self.TwistStampedMsg.header.seq = 0
        self.TwistStampedMsg.header.frame_id = "/cf4"


        listener = tf.TransformListener()

        rospy.sleep(1)

        pos_rot_tuple = listener.lookupTransform('/world', cf_name, rospy.Time.now()-VICON_INTERVAL)
        pos_rot_tuple_old = pos_rot_tuple


        while not rospy.is_shutdown():
            try:
                pos_rot_tuple = listener.lookupTransform('/world', '/cf4', rospy.Time.now()-VICON_INTERVAL)
                x = full_state_from_tf(pos_rot_tuple, pos_rot_tuple_old, VICON_INTERVAL)

                self.PublishFullStateMsgFromNP(x)
                self.PublishTwistStampedMsgFromNP(x)
                rospy.logwarn(x)
                pos_rot_tuple_old = pos_rot_tuple

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass




    def PublishFullStateMsgFromNP(self, x : np.ndarray) -> FullState13:
        """
        Sends a FullState13 message.
        :param x: 13-element np.ndarray
        """

        self.FullStateMsg.header.stamp = rospy.Time.now()
        self.FullStateMsg.header.seq += 1
        self.FullStateMsg.pose.position.x    = x[0]
        self.FullStateMsg.pose.position.y    = x[1]
        self.FullStateMsg.pose.position.z    = x[2]
        self.FullStateMsg.twist.linear.x     = x[7]
        self.FullStateMsg.twist.linear.y     = x[8]
        self.FullStateMsg.twist.linear.z     = x[9]

        self.FullStateMsg.pose.orientation   = geometry_msgs.msg.Quaternion(x[3], x[4], x[5], x[6])
        self.FullStateMsg.twist.angular.x    = x[10]
        self.FullStateMsg.twist.angular.y    = x[11]
        self.FullStateMsg.twist.angular.z    = x[12]
        self.FullStatePublisher.publish(self.FullStateMsg)

    def PublishTwistStampedMsgFromNP(self, x : np.ndarray) -> FullState13:
        """
        Sends a TwistStamped message created from the full state np.array.
        :param x: 13-element np.ndarray
        """

        self.TwistStampedMsg.header.stamp = rospy.Time.now()
        self.TwistStampedMsg.header.seq += 1

        #self.TwistStampedMsg.twist.linear.x     = x[7]
        #self.TwistStampedMsg.twist.linear.y     = x[8]
        #self.TwistStampedMsg.twist.linear.z     = x[9]
        #self.TwistStampedMsg.twist.angular.x    = x[10]
        #self.TwistStampedMsg.twist.angular.y    = x[11]
        #self.TwistStampedMsg.twist.angular.z    = x[12]
        self.TwistStampedMsg.wrench.force.x     = x[7]
        self.TwistStampedMsg.wrench.force.y     = x[8]
        self.TwistStampedMsg.wrench.force.z     = x[9]
        self.TwistStampedMsg.wrench.torque.x    = x[10]
        self.TwistStampedMsg.wrench.torque.y    = x[11]
        self.TwistStampedMsg.wrench.torque.z    = x[12]
        self.TwistStampedPublisher.publish(self.TwistStampedMsg)

def full_state_from_tf(pos_rot_tuple, pos_rot_tuple_old, dt) -> np.ndarray:
    """ 
    Get 13-element state vector from current and previous tf in the form of 
        [x, y, z, qw, qx, qy, qz, vx, vy, vz, rx, ry, rz ]
    :param: pos_rot_tuple: tuple of (position, rotation) from tf
    :param: pos_rot_tuple_old: tuple of (position, rotation) from tf
    :param: dt: time difference between current and previous tf
    :return: 13-element np.ndarray
    """
    pos = np.array(pos_rot_tuple[0])
    rot = np.array(pos_rot_tuple[1])
    pos_old = np.array(pos_rot_tuple_old[0])
    rot_old = np.array(pos_rot_tuple_old[1])
    twist_vel, twist_rot = get_vel(pos, rot, pos_old, rot_old, dt)

    state = np.array([pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3], twist_vel[0], twist_vel[1], twist_vel[2], twist_rot[0], twist_rot[1], twist_rot[2]])
    return state


def get_vel(trans, rot, trans_old, rot_old, dt) -> tuple:
    """
    Calculate the velocity of the drone from the current and previous position and orientation.
    vel_calc_func handles the actual computation.
    :param: trans: current position
    :param: rot: current orientation
    :param: trans_old: previous position
    :param: rot_old: previous orientation
    :param: dt: time difference between current and previous tf
    """
    start_frame = Frame(Rotation.Quaternion(rot_old[0], rot_old[1], rot_old[2], rot_old[3]), Vector(trans_old[0], trans_old[1], trans_old[2]))
    end_frame = Frame(Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]),
            Vector(trans[0], trans[1], trans[2]))


    twist_vel, twist_rot = vel_calc_func(start_frame, end_frame, VICON_INTERVAL)
    return twist_vel, twist_rot

def vel_calc_func(start_frame, end_frame, corrected_average_interval):
    """
    Calculate the velocity of the drone from the current and previous frame.
    This function handles the actual computation (Euler differentiation). 
    """


    # Transform matrices using inverse for correct orientation
    temp = start_frame.M.Inverse() * end_frame.M

    ang, temp_axis = Rotation.GetRotAngle(temp)
    o = start_frame.M * temp_axis
    p_start_vec = start_frame.p
    p_end_vec = end_frame.p
    # print "p vectors ", p_end_vec, p_start_vec
    delta_x = p_end_vec[0] - p_start_vec[0]

    delta_y = p_end_vec[1] - p_start_vec[1]
    delta_z = p_end_vec[2] - p_start_vec[2]

    # Assign values to a Vector3 element
    twist_vel = geometry_msgs.msg.Vector3()
    twist_vel.x = delta_x / corrected_average_interval.to_sec()
    twist_vel.y = delta_y / corrected_average_interval.to_sec()
    twist_vel.z = delta_z / corrected_average_interval.to_sec()
    # twist_rot = geometry_msgs.msg.Vector3()
    twist_rot = o* (ang/corrected_average_interval.to_sec())

    twist_vel = np.array([twist_vel.x, twist_vel.y, twist_vel.z])
    twist_rot = np.array([twist_rot.x(), twist_rot.y(), twist_rot.z()])


    return twist_vel, twist_rot


if __name__ == "__main__":
    node = FullState13PublisherNode()
    rospy.spin()