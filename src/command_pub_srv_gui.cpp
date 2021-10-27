#include "ros/ros.h"
#include <std_msgs/String.h>
#include "navigation_controller/command.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>

class CommandPubSrvGui {
public:
    CommandPubSrvGui();
    ~CommandPubSrvGui();

private:
    ros::NodeHandle nh_;
    ros::Subscriber place_cmd_;
    ros::ServiceClient client_;
    ros::Publisher reach_pub_;
    ros::Publisher flag_switch;
    std_msgs::Bool cmd_switch;

    void get_place(const std_msgs::String::ConstPtr& place);
};

CommandPubSrvGui::CommandPubSrvGui()
{
    place_cmd_ = nh_.subscribe < std_msgs::String > ("place", 1000, boost::bind(&CommandPubSrvGui::get_place, this, _1));
    client_ = nh_.serviceClient < navigation_controller::command > ("pos_cmd");

    flag_switch = nh_.advertise < std_msgs::Bool > ("yourturn", 1);
}

CommandPubSrvGui::~CommandPubSrvGui()
{}

void CommandPubSrvGui::get_place(const std_msgs::String::ConstPtr& place)
{
    int num = 0;
    int flag = 0;
    int wait = 0;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    navigation_controller::command srv;

    std::string path = "/home/locobot/low_cost_ws_my/src/pyrobot/robots/LoCoBot/navigation_controller/cmd_txt/cmd" + place->data + ".txt";
    std::fstream myfile;

    myfile.open(path.c_str());

    while (myfile >> num && ros::ok()) {
        x = 0.0;
        y = 0.0;
        theta = 0.0;

        if (num == 1) {
            myfile >> theta;
            if (flag == 0) {
                wait = 1.5;
                flag = 1;
            } else {
                wait = 0;
                flag = 0;
            }
        } else if (num == 2) {
            myfile >> x >> y;
            wait = 0;
        } else if (num == 0) {
            continue;
        }
        ROS_INFO_STREAM("x = " << x << " y = " << y << " theta = " << theta);
        srv.request.type = num;
        srv.request.x = x;
        srv.request.y = y;
        srv.request.theta = theta;

        if (client_.call(srv)) {
            ROS_INFO("call service success");
            srv.request.type = 0;
            while (srv.response.run_completed == false) {
                usleep(100000);
                ROS_INFO("hello servie");
                client_.call(srv);
            }
            sleep(wait);
        } else {
            ROS_INFO("call service fail");
        }
    }
    cmd_switch.data = true;
    for (int i = 0; i < 3; i++) {
        flag_switch.publish(cmd_switch);
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_pub_srv_gui");
    CommandPubSrvGui commandpubsrvgui;

    ros::spin();
    return 0;
}
