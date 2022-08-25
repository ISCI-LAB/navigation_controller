import rospy
from navigation_controller.srv import command
import tf
import numpy as np


class SampleClient(object):
    def __init__(self):
        rospy.wait_for_service('pos_cmd')
        self.goal_service = rospy.ServiceProxy('pos_cmd', command)

    # keep send request to server for checking return value
    def loop_service_confirm(self):
        goal_achived = False

        while not goal_achived:
            try:
                resp = self.goal_service(0, 0, 0, 0)
                goal_achived = resp.run_completed
            except rospy.ServiceException, e:
                print("Service call failed: %s", e)

        return goal_achived

    def pos_cmd_request(self, request_type, request_x, request_y, request_theta):
        try:
            resp = self.goal_service(request_type, request_x, request_y, request_theta)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)

    # two stage navigation, first transition and then rotation
    def send_goal(self, goal_x, goal_y, goal_theta):
        self.pos_cmd_request(2, goal_x, goal_y, 0)
        goal_achived = self.loop_service_confirm()

        if not goal_achived:
            print("Fail at type 2 in Navigation")
            return False

        self.pos_cmd_request(1, 0, 0, goal_theta)
        goal_achived = self.loop_service_confirm()

        return goal_achived


if __name__ == '__main__':
    rospy.init_node('sample_client_node', anonymous=False)
    sample_client_node = SampleClient()
    rospy.spin()
