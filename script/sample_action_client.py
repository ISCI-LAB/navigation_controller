import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import actionlib
import tf


class SampleClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "navigation_controller/send_goal",
            MoveBaseAction)
        self.client.wait_for_server()

    def async_send_goal(self, goal_x, goal_y, theta):
        theta_quat = tf.transformations.quaternion_from_euler(0., 0., theta)
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.
        goal.target_pose.pose.orientation.x = theta_quat[0]
        goal.target_pose.pose.orientation.y = theta_quat[1]
        goal.target_pose.pose.orientation.z = theta_quat[2]
        goal.target_pose.pose.orientation.w = theta_quat[3]
        self.client.send_goal(goal)
        print("wait for result")
        self.client.wait_for_result()
        print("goal reach success")


if __name__ == '__main__':
    rospy.init_node('sample_client_node', anonymous=False)
    sample_client_node = SampleClient()
    rospy.spin()
