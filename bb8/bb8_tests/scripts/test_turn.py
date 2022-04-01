#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

class BB8Tests(object):

    def __init__(self):

        #odom_pub = rospy.Publisher('/cmd_vel', Pose)
        #rospy.Subscriber("/odom", Odometry, self._odom_callback)
        # We will compare odometry values with the real simulation values
        self._model = GetModelStateRequest()
        self._model.model_name = 'bb_8'
        rospy.wait_for_service('/gazebo/get_model_state')
        self._get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def _odom_callback(self,msg):
        pass
    def _get_odom(self):
        rospy.loginfo("Getting Odom")
        current_odom = None
        while current_odom is None and not rospy.is_shutdown():
            try:
                current_odom = rospy.wait_for_message("/odom", Odometry, timeout=1.0)
            except:
                rospy.logerr("Current /odom not ready yet, retrying for getting joint_states")
        rospy.loginfo("Getting Odom..DONE")
        return current_odom


    def _compare_two_poses(self,pose1, pose2):
        """
        It compares to give Pose objects and tells the difference
        :param pose1:
        :param pose2:
        :return:
        """

        rospy.loginfo("#######POS 1 ##########")

        orientation_q1 = pose1.orientation
        orientation1 = [orientation_q1.x,
                            orientation_q1.y,
                            orientation_q1.z,
                            orientation_q1.w]
        (roll1, pitch1, yaw1) = euler_from_quaternion (orientation1)
        rospy.loginfo("Orientation1=["+str(roll1)+","+str(pitch1)+","+str(yaw1)+"]")
        rospy.loginfo("#######################")

        rospy.loginfo("#######POS 2 ##########")
        orientation_q2 = pose2.orientation
        orientation2 = [orientation_q2.x,
                            orientation_q2.y,
                            orientation_q2.z,
                            orientation_q2.w]
        (roll2, pitch2, yaw2) = euler_from_quaternion (orientation2)
        rospy.loginfo("Orientation1=["+str(roll2)+","+str(pitch2)+","+str(yaw2)+"]")
        rospy.loginfo("#######################")

        # TODO: return difference
        return None

    def test_turn(self):
        """
        This function tests the accuracy of bb8 odometry turning
        :return:
        """

        r = rospy.Rate(2)

        while not rospy.is_shutdown():

            rospy.loginfo("Getting model state...")
            result = self._get_model_srv(self._model)
            rospy.loginfo("Getting model state...DONE")


            odom_now = self._get_odom()
            self._compare_two_poses(odom_now.pose.pose, result.pose)

            r.sleep()

    def test_turn_cmd(self):
        """
        This function tests the accuracy of bb8 odometry turning manually
        :return:
        """

        r = rospy.Rate(2)
        move_time = 1.6
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.0
        self._pub.publish(cmd_vel_msg)
        time.sleep(move_time)

        while not rospy.is_shutdown():

            cmd_vel_msg.angular.z = 0.5
            self._pub.publish(cmd_vel_msg)
            time.sleep(move_time)
            cmd_vel_msg.angular.z = 0.0
            self._pub.publish(cmd_vel_msg)
            time.sleep(move_time)

            rospy.loginfo("Getting model state...")
            result = self._get_model_srv(self._model)
            rospy.loginfo("Getting model state...DONE")


            odom_now = self._get_odom()
            self._compare_two_poses(odom_now.pose.pose, result.pose)

            raw_input("Press Key for next step...")


if __name__ == "__main__":

    rospy.init_node('bb8_turn_test_node', log_level=rospy.DEBUG)
    bb8_tests_obj = BB8Tests()
    #bb8_tests_obj.test_turn()
    bb8_tests_obj.test_turn_cmd()
