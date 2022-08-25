#!/usr/bin/python

import rospy
from navigation_controller.srv import command
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
import tf2_ros
from tf.transformations import euler_from_quaternion
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped
import math


class NavigationServer(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(5)
        rospy.wait_for_service('pos_cmd')
        self.goal_service = rospy.ServiceProxy('pos_cmd', command)
        self.as_ = actionlib.SimpleActionServer(
            "navigation_controller/send_goal",
            MoveBaseAction,
            execute_cb=self.execute_cb, auto_start=False)
        self.map_frame_id = rospy.get_param("/calculat_navigation_cmd/map_frame_id")
        self.as_.start()

    def execute_cb(self, move_base_goal):
        goal = move_base_goal.target_pose
        goal_quat = goal.pose.orientation
        goal_angle = euler_from_quaternion([goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w])

        print("Move to next goal: [{}, {}, {}]".format(goal.pose.position.x, goal.pose.position.y, goal_angle[2]))
        skip_linear = self.check_nearby(goal)
        success = self.send_goal(goal.pose.position.x, goal.pose.position.y, goal_angle[2], skip_linear=skip_linear)
        if success:
            print("Goal reached!!!")
            self.as_.set_succeeded(MoveBaseResult(), "Goal reached!")

    def check_nearby(self, new_goal):
        current_pose = self.get_robot_position()
        dist = math.sqrt((current_pose.pose.position.x - new_goal.pose.position.x)**2 +
                         (current_pose.pose.position.y - new_goal.pose.position.y)**2)
        return dist < 0.025

    def get_robot_position(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.map_frame_id, "base_link", rospy.Time())
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)

    # keep send request to server for checking return value
    def loop_service_confirm(self):
        goal_achived = False

        while not goal_achived:
            try:
                resp = self.goal_service(0, 0, 0, 0)
                goal_achived = resp.run_completed
                robot_pos = self.get_robot_position()

                if isinstance(robot_pos, PoseStamped):
                    feedback = MoveBaseFeedback()
                    feedback.base_position = robot_pos
                    self.as_.publish_feedback(feedback)
            except rospy.ServiceException, e:
                print("Service call failed: %s", e)

            self.rate.sleep()

        return goal_achived

    def pos_cmd_request(self, request_type, request_x, request_y, request_theta):
        try:
            resp = self.goal_service(request_type, request_x, request_y, request_theta)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)

    def send_goal(self, goal_x, goal_y, goal_theta, skip_linear=False):
        if not skip_linear:
            current_pose = self.get_robot_position()
            relative_theta = math.atan2((goal_y - current_pose.pose.position.y), (goal_x - current_pose.pose.position.x))
            self.pos_cmd_request(1, 0, 0, relative_theta)
            goal_achived = self.loop_service_confirm()

            if not goal_achived:
                print("Fail at type 1 in Navigation")
                return False

            self.pos_cmd_request(2, goal_x, goal_y, 0)
            goal_achived = self.loop_service_confirm()

            if not goal_achived:
                print("Fail at type 2 in Navigation")
                return False

        self.pos_cmd_request(1, 0, 0, goal_theta)
        goal_achived = self.loop_service_confirm()

        return goal_achived


if __name__ == '__main__':
    rospy.init_node('navigation_server_node', anonymous=False)
    sample_client_node = NavigationServer()
    rospy.spin()
