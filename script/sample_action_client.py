import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped


class SampleClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "navigation_controller/send_goal",
            MoveBaseAction)
        self.client.wait_for_server()
        self.goal_sub = rospy.Subscriber('demo_goal', PoseStamped, self.demo_cb, queue_size=1)

    def demo_cb(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.client.send_goal(goal)
        print("wait for result")
        self.client.wait_for_result()
        print("goal reach success")


if __name__ == '__main__':
    rospy.init_node('sample_client_node', anonymous=False)
    sample_client_node = SampleClient()
    rospy.spin()
