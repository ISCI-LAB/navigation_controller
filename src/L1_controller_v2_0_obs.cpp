/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
//******************************1201
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include <costmap_2d/costmap_2d_ros.h>
//******************************1201
#include <visualization_msgs/Marker.h>
//1212
#include <std_msgs/Bool.h>
//1212
#include <move_base_msgs/MoveBaseActionResult.h> // "move_base/result"
#include <tf/transform_listener.h> // "amcl_pose"

//******************************fusion
#include "navigation_controller/command.h"
#include <iostream>
#include <fstream>
#include <actionlib_msgs/GoalID.h>
//******************************fusion

// obstacle avoidance
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
    public:
        L1Controller();
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta);
        double getGasInput(const float& current_v);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
	      double getStandingAngle(double currentTheta);

	//******************************1201
	void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
				
	void obstacle_scan();
	//******************************1201

    private:
        ros::NodeHandle n_;
	//******************************fusion
        ros::Subscriber odom_sub, path_sub, goal_sub,tag_goal_sub;
	ros::Publisher odom_initial_pub;
	geometry_msgs::PoseStamped tag_goal;
	std_msgs::Bool odom_initial_flag;
	ros::ServiceClient client_;
	 navigation_controller::command srv;
	bool cmd_flag;
	void taggoalCB(const geometry_msgs::PoseStamped::ConstPtr& taggoalMsg);
	void get_place();
	
	//******************************fusion

        //ros::Subscriber odom_sub, path_sub, goal_sub;
        ros::Publisher pub_, marker_pub;
	ros::Publisher result_; //// "move_base/result"
	//**********************************s
	ros::Publisher result_waypt; //// "move_base/result"
	//**********************************s
	
	//**********1025****************
	ros::Publisher pub_pose;
	geometry_msgs::Pose position;
	//******************************1025

	//******************************1201
	
	//1215
	ros::Publisher cancel_pub_;
	actionlib_msgs::GoalID first_goal;
	ros::Subscriber movebase_result;
	void movebaseCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& resultMsg);
	bool result_flag;
	//1215

	//1209
	ros::Publisher goal_pose;
	int count=0;
	geometry_msgs::PoseStamped odom_goal_revise;
	bool path_recieved=false;
	//1209
 
        //1212
	ros::Publisher obstacle_clear;
	std_msgs::Bool obstacle_flag;
	int count_obs=0;
	//1212
	
	//1213
	double obs_lfw;
	//1213
	ros::Subscriber costmap_sub;
	nav_msgs::OccupancyGrid costmap;
	double car_r;
		
	double costmap_width ;	
	double costmap_height ;
	double costmap_totalnum ;	
	double costmap_resolution ;

	bool costmap_received;
	bool obs_pub_flag;
	//******************************1201
        ros::Timer timer1, timer2, timer3;
        tf::TransformListener tf_listener;

        visualization_msgs::Marker points, line_strip, goal_circle;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point odom_goal_pos;
	geometry_msgs::Quaternion odom_goal_ori;

        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path;

        double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
        double Gas_gain, baseAngle, Angle_gain, goalRadius, baseSpeed;
        int controller_freq;
        bool foundForwardPt, goal_received, goal_reached, goal_reached_a;
	int smooth_counter;
	double goal2origin_theta;
	int direct=0;
	int delete_point=0;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);

	//obstacle avoidance
	static const int beam_num;
	double kAvoidRadius_;
	sensor_msgs::LaserScan scan_;
	visualization_msgs::MarkerArray mark_array_;
	ros::Subscriber scan_sub_;
	ros::Publisher mark_array_pub_;
	void get_scan(const sensor_msgs::LaserScan::ConstPtr& scan);
	void get_obstacle_vector(const ros::TimerEvent&);
	double obs_r;
	double obs_theta;
	double minrange;
        geometry_msgs::Twist SafeVelocity(geometry_msgs::Twist cmd_vel);

}; // end of class

const int L1Controller::beam_num = 30;


L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("~");
    //******************************fusion
    tag_goal_sub = n_.subscribe("tag_goal", 1, &L1Controller::taggoalCB, this);	
    odom_initial_pub =n_.advertise<std_msgs::Bool>("odom_initial_flag", 5);
    client_ = n_.serviceClient<navigation_controller::command>("robot_0/pos_cmd");
    //navigation_controller::command srv;
    cmd_flag=true;
    //******************************fusion

    //******************************1201
    costmap_sub = n_.subscribe("/move_base/global_costmap/costmap", 1, &L1Controller::costmapCB,this);
    car_r=0.45;//m
    costmap_received=false;
    obs_pub_flag=false; 
    //******************************1201


    //Car parameter
    pn.param("L", L, 0.26);
    pn.param("Lrv", Lrv, 10.0);
    pn.param("Vcmd", Vcmd, 0.4);
    pn.param("lfw", lfw, 0.13);
    pn.param("lrv", lrv, 10.0);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 0.2);
    pn.param("baseAngle", baseAngle, 0.0);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    //******************1025*****************
    pub_pose = n_.advertise<geometry_msgs::Pose>("robot_0/pose", 1);
    //***************************************
    result_ = n_.advertise<move_base_msgs::MoveBaseActionResult>("move_base/result_", 10); //// "move_base/result"
    //**********************************s
    result_waypt = n_.advertise<move_base_msgs::MoveBaseActionResult>("move_base/result_waypt", 10);
    //**********************************s


    //1215
    cancel_pub_ =  n_.advertise<actionlib_msgs::GoalID>("/robot_0/move_base/cancel",1);
    movebase_result = n_.subscribe("/robot_0/move_base/result", 10, &L1Controller::movebaseCB, this);
    result_flag = false;
    //1215


    //1209
    goal_pose = n_.advertise<geometry_msgs::PoseStamped>("robot_0/goal", 1);
    //1209
	
    //1212
    obstacle_flag.data=false;
    obstacle_clear = n_.advertise<std_msgs::Bool>("robot_0/obstacle_clear", 1);
    //1212
    
    //1213
    obs_lfw=0.4;
    //1213

    //Timer
    timer1 = n_.createTimer(ros::Duration((0.05)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.1)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    //Lfw = goalRadius = getL1Distance(Vcmd);
    Lfw = 0.8;
    goalRadius = 0.1;
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    goal_reached_a = false;
    cmd_vel.linear.x = 0; // 0 for stop
    cmd_vel.angular.z = 0;
    smooth_counter = 0;

    //Show info
    ROS_INFO("[param] baseSpeed: %f", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();

    //Obstacle Avoidance
    obs_r = 0.0;
    obs_theta = 0.0;
    minrange = 0.0;
    kAvoidRadius_ = 0.5;
    scan_sub_ = n_.subscribe("scan", 1, &L1Controller::get_scan, this);
    mark_array_pub_ = n_.advertise<visualization_msgs::MarkerArray>("mark_array", 1 );
    timer3 = n_.createTimer(ros::Duration((0.1)/controller_freq), &L1Controller::get_obstacle_vector, this); // Duration(0.05) -> 20Hz
}


//obstacle avoidance
void L1Controller::get_scan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	scan_ = *scan;
}

void L1Controller::get_obstacle_vector(const ros::TimerEvent&)
{
	int beam_range = scan_.ranges.size() / beam_num;
	double beam_minrange[beam_num];		//m
	double beam_minrange_2[beam_num];	//m
	double beam_angle[beam_num];		//rad
	double adjust;				//m
	double adjust_2;			//m

	minrange = scan_.range_max;		//m
	for(int i = 0; i < beam_num; i++) {
		if(i >= (beam_num / 4) && i <= (beam_num * 3 / 4)){	// beam_num / 5 && beam_num* 4 / 5

			adjust = 0.35;
			adjust_2 = 0.0;
		}
		else{
			adjust = 0.3;	//0.3
			adjust_2 = 0.3;
		}
		beam_minrange[i] = scan_.range_max;
		beam_minrange_2[i] = scan_.range_max;
		for(int j = i * beam_range; j < (i+1) * beam_range; j++) {
			if(std::isnormal(scan_.ranges[j])){
				beam_minrange[i] = std::min(beam_minrange[i],(double)std::max((scan_.ranges[j]-adjust), 0.0));
				beam_minrange_2[i] = std::min(beam_minrange_2[i],(double)std::max((scan_.ranges[j]-adjust_2), 0.0));
			}
		}
		//ROS_INFO_STREAM("beam_minrange_1 is " << beam_minrange[i]);
		//ROS_INFO_STREAM("beam_minrange_2 is " << beam_minrange_2[i]);
		/*
		if(beam_minrange[i]==0.0 && beam_minrange_2[i]!=0.0){

			beam_minrange[i] = 0.1;
		}*/
		beam_angle[i] = scan_.angle_min + ((double)i + 0.5) * (double)beam_range * scan_.angle_increment;
		//ROS_INFO_STREAM("angle " << i << ": " << beam_angle[i] * R2D << " min : " << beam_minrange[i]);
		if(beam_minrange[i] != 0.0&& beam_minrange[i] > 0.05)
			minrange = std::min(minrange,beam_minrange[i]);
	}
	//-------------------------------------------------
	double obs_vector_x = 0.0;
	double obs_vector_y = 0.0;

	int obs_count = 0;
	for (int i = 0; i < beam_num; i++) {
		double temp = 0.0;
		if (beam_minrange[i] < kAvoidRadius_ && beam_minrange[i] > 0.05) {
			temp = 1.0 - pow((beam_minrange[i] / kAvoidRadius_),0.5);
			obs_vector_x += temp * cos(beam_angle[i]);
			obs_vector_y += temp * sin(beam_angle[i]);
			obs_count++;
		}
	}

	obs_r = 0.0;
	obs_theta = 0.0;
	if(obs_count > 0) {
		obs_vector_x /= obs_count;
		obs_vector_y /= obs_count;
		obs_r = sqrt(pow(obs_vector_x, 2) + pow(obs_vector_y, 2));
		obs_theta = atan2(obs_vector_y, obs_vector_x);
	}
	//ROS_INFO_STREAM("obs_count " << obs_count << " obs_r " << obs_r << " obs_theta : " << obs_theta);
	//------------------------------------------------
	mark_array_.markers.clear();

	visualization_msgs::Marker marker;
	marker.header.frame_id = "robot_0/base_laser_link";
	marker.header.stamp = ros::Time::now();
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	marker.color.r = 0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;


	for(int i = 0; i < beam_num; i++) {
		char temp[50];
		sprintf(temp,"obs_arrow%d",i);
		marker.ns = std::string(temp);
		marker.pose.orientation.z = sin(beam_angle[i]/2);
		marker.pose.orientation.w = cos(beam_angle[i]/2);
		if(beam_minrange[i] <= 0)
			marker.scale.x = 0.00001;
		else
			marker.scale.x = beam_minrange[i];
		mark_array_.markers.push_back(marker);
	}

	marker.ns = "obs_arrow";
	marker.pose.orientation.z = sin(obs_theta/2);
	marker.pose.orientation.w = cos(obs_theta/2);
	if(obs_r <= 0)
		marker.scale.x = 0.00001;
	else
		marker.scale.x = obs_r;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	mark_array_.markers.push_back(marker);
	mark_array_pub_.publish(mark_array_);
}

geometry_msgs::Twist L1Controller::SafeVelocity(geometry_msgs::Twist cmd_vel){

	double Ea = 1;
	double Ea_s = 0.85;
	double Ea_d = 0.5;
	Ea = pow(std::min(pow(minrange / kAvoidRadius_,2), 1.0), 0.2);
	//Ea=1;
	//ROS_ERROR("Ea: %.2f",Ea);
	if(Ea >= Ea_s){
		//1213
    		obs_lfw=0.4;
    		//1213
	}else if(Ea >= Ea_d){
		cmd_vel.linear.x = ((Ea - Ea_d)/(Ea_s-Ea_d))*cmd_vel.linear.x;
		//1213
		obs_lfw=0.4;
	}	
	else{
		cmd_vel.linear.x = 0.0;
		//cmd_vel.angular.z = cmd_vel.angular.z*0.5;
		//1213
		obs_lfw=0.4;
	}
	return cmd_vel;
}
//1215
void L1Controller::movebaseCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& resultMsg) {

	//first_goal.stamp=resultMsg->status.goal_id.stamp;
	first_goal.id=resultMsg->status.goal_id.id;
	ROS_ERROR("ANANANANANANANANANANANANANA");
	result_flag = true;


}

//1215
void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "/robot_0/odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}
//******************************1201

void L1Controller::costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	
	costmap=*msg;
	costmap_received=true;
	//ROS_ERROR("fuck");
	float x,y; 
	
	
	//ros::NodeHandle nh_;
	//ros::Publisher mark_array_pub_= n.advertise<visualization_msgs::MarkerArray>("mark_array", 1 );
	//std::string path_0 = "/home/nvidia/catkin_ws_nav/src/navigation_controller/cmd_txt/cmd666.txt";
	//visualization_msgs::MarkerArray mark_array_;
	

	//std_msgs::String place_0;
	//std::ofstream out_0(path_0);
	
	costmap_width = msg->info.width;	
	costmap_height = msg->info.height;
	costmap_totalnum = costmap_width*costmap_height;	
	costmap_resolution = msg->info.resolution;
	
	obstacle_scan();
	for(int h=0; h<costmap_height; h++)
	{
		for(int w=0; w<costmap_width; w++)
		{
			//x=(w)*costmap_resolution;
			//y=(h)*costmap_resolution;
			//out_0<<"2 "<<std::to_string(x*(-1))<<" "<<std::to_string(y-costmap_height*costmap_resolution)<<std::endl;
			if(msg->data[h*costmap_width+w]>=90)
			{
				x=msg->info.origin.position.x+(w+0.5)*costmap_resolution;
				y=msg->info.origin.position.y+(h+0.5)*costmap_resolution;
				//costmap_2d::Costmap2D::worldToMap
				//worldToMap(x*(-1),y-costmap_height*costmap_resolution,mx,my);
				//out_0<<"2 "<<std::to_string(x*(-1))<<" "<<std::to_string(y-costmap_height*costmap_resolution)<<std::endl;
				//obstacle_list.push_back(RVO::Vector2(x, y));
				//out_0<<std::to_string(x)<<" "<<std::to_string(y)<<std::endl;
				//ROS_ERROR("x:%.2f  y:%.2f  ", temp.x(), temp.y());
				//ROS_ERROR("obstacle: %zu", obstacle_list.size());
				//ROS_ERROR("obstacle: %zu", points_array.markers.size());
				
			}			
		}
	}

	
}

void L1Controller::obstacle_scan(){
	ros::Rate rate(10);

	while (ros::ok()) {

	double x;
	double y;
	int w;
	int h;
	int obs_num = 0.0;
	//cmap->getCostmap();
    //odom_goal_pos.z
	costmap_width = costmap.info.width;	
	costmap_height =  costmap.info.height;
	costmap_totalnum = costmap_width*costmap_height;	
	costmap_resolution =  costmap.info.resolution;
	//ROS_ERROR("costmap_width: %.2f",costmap_width);
	//ROS_ERROR("costmap_height: %.2f",costmap_height);
	//ROS_ERROR("costmap_resolution: %.2f",costmap_resolution);
	//ROS_ERROR("costmap_width: %.2f",costmap_width]);
	//ROS_ERROR("info: %d",sizeof(costmap.data));
	if(odom_goal_pos.z==1&&costmap_received==true){
		
		for(float i=-(car_r);i<=(car_r);i+=costmap_resolution){
			y=odom_goal_pos.y-(car_r);
			x=odom_goal_pos.x+i;
			w=((x- costmap.info.origin.position.x)/costmap_resolution)-0.5;
			h=((y- costmap.info.origin.position.y)/costmap_resolution)-0.5;
			//ROS_ERROR("x: %.2f",x);
			//ROS_ERROR("y: %.2f",y);
			//ROS_ERROR("w: %d",w);
			//ROS_ERROR("h: %d",h);
			if( costmap.data[(h-1)*costmap_width+w]>=50)
				obs_num++;
				
		}
		for(float i=-(car_r);i<=(car_r);i+=0.05){
			y=odom_goal_pos.y+(car_r);
			x=odom_goal_pos.x+i;
			w=((x- costmap.info.origin.position.x)/costmap_resolution)-0.5;
			h=((y- costmap.info.origin.position.y)/costmap_resolution)-0.5;
			if( costmap.data[(h-1)*costmap_width+w]>=50)
				obs_num++;
		}
		for(float i=-(car_r);i<=(car_r);i+=0.05){
			y=odom_goal_pos.y+i;
			x=odom_goal_pos.x-(car_r);
			w=((x- costmap.info.origin.position.x)/costmap_resolution)-0.5;
			h=((y- costmap.info.origin.position.y)/costmap_resolution)-0.5;
			if( costmap.data[(h-1)*costmap_width+w]>=50)
				obs_num++;
		}
		for(float i=-(car_r);i<=(car_r);i+=0.05){
			y=odom_goal_pos.y+i;
			x=odom_goal_pos.x+(car_r);
			w=((x- costmap.info.origin.position.x)/costmap_resolution)-0.5;
			h=((y- costmap.info.origin.position.y)/costmap_resolution)-0.5;
			if( costmap.data[(h-1)*costmap_width+w]>=50)
				obs_num++;
		}
		//ROS_ERROR("%d ",obs_num);
		//ROS_ERROR("%d , %d",obs_num,goalID);
		if(obs_num>9 &&obs_pub_flag==false){//1204
			//goalID++;
			//if(odom_goal_pos)
			obs_pub_flag=true;
			std::cout<<"x: "<<odom_goal_pos.x<<"   y: "<<odom_goal_pos.y<<std::endl;
			//1213
			obs_lfw=0.6;
			std::cout<<"0////////////////////////////////////////////////////////"<<std::endl;
			move_base_msgs::MoveBaseActionResult goalresult;
	    			goalresult.header.stamp = ros::Time::now();
	    			goalresult.header.frame_id = "/map";
	    			goalresult.status.status = 2;
	    			result_waypt.publish(goalresult);
		}
	}

	ros::spinOnce();
	rate.sleep();



	}


}

//******************************1201

void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) // "amcl_pose"
{
    //get robot position
    tf::StampedTransform transform;
    tf::Quaternion q;

    try {
        tf_listener.lookupTransform("/map", "/robot_0/base_footprint", ros::Time(0), transform);
	q = transform.getRotation();
    }
    catch (tf::TransformException ex) {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();
    }
    odom = *odomMsg;
    odom.pose.pose.position.x = transform.getOrigin().x();
    odom.pose.pose.position.y = transform.getOrigin().y();
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(q));
    //************1025**************
    position.position.x=transform.getOrigin().x();
    position.position.y=transform.getOrigin().y();
    position.orientation=tf::createQuaternionMsgFromYaw(tf::getYaw(q));
    pub_pose.publish(position);
    //******************************
    //ROS_INFO("position.x = %.2f",odom.pose.pose.position.x);
    //ROS_INFO("position.y = %.2f",odom.pose.pose.position.y);
}


void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}
//******************************fusion
void L1Controller::taggoalCB(const geometry_msgs::PoseStamped::ConstPtr& taggoalMsg){	
    tag_goal = *taggoalMsg;	
    ROS_ERROR("x: %.2f  y: %.2f  z: %.2f",tag_goal.pose.position.x,tag_goal.pose.position.y,tag_goal.pose.position.z);	
}
void L1Controller::get_place()
{
	int num = 0;
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	navigation_controller::command srv;

	std::string path = "/home/nvidia/syntech_multi_0/src/navigation_controller/cmd_txt/cmd00.txt";
	std::fstream myfile;
	myfile.open(path.c_str());

	while (myfile >> num && ros::ok()) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		first_goal.stamp = ros::Time::now();
		cancel_pub_.publish(first_goal);
		if(num == 1 || num == 4 || num == 6|| num == 9)
			myfile >> theta;
		else if(num == 2 || num == 3 || num == 5|| num == 7|| num == 8)
			myfile >> x >> y;
		else if(num == 0) {
			continue;
		}
	
		ROS_INFO_STREAM("x = " << x << " y = " << y << " theta = " << theta);
		
		srv.request.type = num;
		srv.request.x = x;
		srv.request.y = y;
		srv.request.theta = theta;
		//srv.request.is_nav = true;

		if(client_.call(srv)) {
			ROS_INFO("call service success");
			srv.request.type = 0;
			while(srv.response.run_completed == false) {
				usleep(100000);
				//ROS_INFO("hello servie");
				client_.call(srv);
			}
		}
		else {
			ROS_INFO("call service fail");
		}
	}
	myfile.close();
	move_base_msgs::MoveBaseActionResult goalresult;
	    			goalresult.header.stamp = ros::Time::now();
	    			goalresult.header.frame_id = "/map";
	    			goalresult.status.status = 3;
	    			result_.publish(goalresult);
	sleep(7);
	std::string path1 = "/home/nvidia/syntech_multi_0/src/navigation_controller/cmd_txt/cmd01.txt";
	std::fstream myfile1;
	myfile1.open(path1.c_str());

	while (myfile1 >> num && ros::ok()) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		first_goal.stamp = ros::Time::now();
		cancel_pub_.publish(first_goal);
		if(num == 1 || num == 4 || num == 6|| num == 9)
			myfile1 >> theta;
		else if(num == 2 || num == 3 || num == 5|| num == 7|| num == 8)
			myfile1 >> x >> y;
		else if(num == 0) {
			continue;
		}
	
		ROS_INFO_STREAM("x = " << x << " y = " << y << " theta = " << theta);
		
		srv.request.type = num;
		srv.request.x = x;
		srv.request.y = y;
		srv.request.theta = theta;
		//srv.request.is_nav = true;

		if(client_.call(srv)) {
			ROS_INFO("call service success");
			srv.request.type = 0;
			while(srv.response.run_completed == false) {
				usleep(100000);
				//ROS_INFO("hello servie");
				client_.call(srv);
			}
		}
		else {
			ROS_INFO("call service fail");
		}
	}
	myfile1.close();
	sleep(1);
        //sleep(5);
}
//******************************fusion

void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
	//1209
	count=0;
	//1209

	//1212
	count_obs=0;
	//1212
	
	//1215
	result_flag =false;
	//1215
        geometry_msgs::PoseStamped odom_goal;
	tf::StampedTransform transform;
        tf::Quaternion q;
        //tf_listener.transformPose("/odom", ros::Time(0) , *goalMsg, "/map" ,odom_goal);
	odom_goal = *goalMsg;
	//1209
	odom_goal_revise=*goalMsg;
	//1209
	odom_goal_pos = odom_goal.pose.position;
	odom_goal_ori = odom_goal.pose.orientation;
        goal_received = true;
        goal_reached = false;
	goal_reached_a = false;
	//************************************************1201
	obs_pub_flag=false;
	//************************************************1201
	 try {
        tf_listener.lookupTransform("/map", "/robot_0/base_footprint", ros::Time(0), transform);
	q = transform.getRotation();
    	}
    	catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
    	}
	goal2origin_theta=atan2((odom_goal_pos.y-transform.getOrigin().y()),(odom_goal_pos.x-transform.getOrigin().x()));
	if((goal2origin_theta>0)&&(goal2origin_theta<M_PI/2)||(goal2origin_theta<0)&&(goal2origin_theta>-M_PI/2))
		direct=1;
	else
		direct=-1;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getStandingAngle(double currentTheta){

	geometry_msgs::Pose pose;
	pose.orientation = odom_goal_ori;

	double goalTheta = getYawFromPose(pose);
	double deltaTheta;

	double kp = 1.0;
	double w = 0.0;

	//angle between -180 ~ 180 degree
	if (currentTheta > PI)
		currentTheta = currentTheta - int((currentTheta + PI) / (2 * PI)) * (2 * PI);
	else
		currentTheta = currentTheta - int((currentTheta - PI) / (2 * PI)) * (2 * PI);

	if (goalTheta > PI)
		goalTheta = goalTheta - int((goalTheta + PI) / (2 * PI)) * (2 * PI);
	else
		goalTheta = goalTheta - int((goalTheta - PI) / (2 * PI)) * (2 * PI);

	deltaTheta = goalTheta - currentTheta;
	//in order not to turn opposite direction
	if (deltaTheta > PI)
		deltaTheta -= 2.0 * PI;
	else if (deltaTheta < -PI)
		deltaTheta += 2.0 * PI;

	w = kp * deltaTheta;
	ROS_INFO("w : %lf",w);
	if (w >= 30.0 / 180.0 * PI)
		w = 30.0 / 180.0 * PI;
	else if (w <= -30.0 / 180.0 * PI)
		w = -30.0 / 180.0 * PI;

	ROS_INFO("w = %lf",w);
	if (-0.025 < w && w < 0.025) // 0.017
		w = 0.0;
	ROS_INFO("w = %lf",w);

	if(odom_goal_pos.z == 1.0)
		w = 0.0;

	return w;
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double theta = atan2(dy,dx);
    double dist = sqrt(dx*dx + dy*dy);
	/*
    if(direct==1){
    if((theta>0)&&(theta<M_PI/2)||(theta<0)&&(theta>-M_PI/2))
		;
		//direct=1;
    else{
		//ROS_INFO("theta: %.2f",theta);
		return false;
	}
    }
    if(direct==-1){
    if((theta>0)&&(theta<M_PI/2)||(theta<0)&&(theta>-M_PI/2))
		//direct=1;
		return false;
    else
		;//return false;
    }
*/
    //ROS_INFO("dist = %.2f when Lfw = %.2f",dist,Lfw);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;
    
    if(!goal_reached){
	//ROS_ERROR("%.2d",map_path.poses.size());
        for(int i =0; i< map_path.poses.size(); i++)
        {
	    //ROS_ERROR("%.2d",i);
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                //tf_listener.transformPose("/odom", ros::Time(0) , map_path_pose, "/map" ,odom_path_pose);
		odom_path_pose = map_path_pose;
		geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                //bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                if(true)
                {
                   // bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
		     if(getCar2GoalDist()<1){
		    	forwardPt = odom_goal_pos;
		    	foundForwardPt = true;
		    }
		    else{
			 bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    	if(_isWayPtAwayFromLfwDist)
		    	//if(true)
                    	{
				//ROS_ERROR("%.2d",i);
                        	forwardPt = odom_path_wayPt;
                        	foundForwardPt = true;
		        	if((i > map_path.poses.size()-5))
			    		Lfw = 0.3;
				else
			    		Lfw = 0.8;
		        	//ROS_INFO("Robot arrived path_wayPt Last no.%lu",map_path.poses.size()-i);
				delete_point=0;
				//ROS_INFO("=====================================================");
                        	break;
                    	}
			//ROS_INFO("fuck you fuck you fuck you fuck you fuck you fuck you");
			else
			{
				delete_point=+map_path.poses[i].pose.position.z;

				//std::cout<<"0      " <<"original:   "<<map_path.poses[i].pose.position.z<<"     "<<delete_point<<std::endl;
			}
		   }
                   
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }
	//ROS_ERROR("=============================");
        //ROS_INFO("path_wayPt is %lf %lf at No.%lu",forwardPt.x,forwardPt.y,map_path.poses.size()-i);
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();
    

    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
	marker_pub.publish(points);
    	marker_pub.publish(line_strip);

    //////////////// motion controller
    	odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    	odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);

	//1209
 	//map_path.poses.clear();
	path_recieved=false;
        //1209
	
    	return odom_car2WayPtVec;
    }/*
    else{
	ROS_INFO("irelia god irelia god irelia god irelia god irelia god irelia god irelia god ");	
    }*/
/*
    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    //////////////// motion controller
    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;*/
}


double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    //ROS_ERROR("%.2f",eta);
    return eta;
}


double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 3 / 3.0; //////////////////////// changed it
        //L1 = 0.2;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{
    double steeringAnge = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    //ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v)*Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}


void L1Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        //ROS_INFO("car2goal_dist = %.2f when goalRadius = %.2f",car2goal_dist, goalRadius);
	if(goal_reached != true){	
		if(odom_goal_pos.z == 1.0){
			//1213
			if(car2goal_dist < obs_lfw){ //Lfw/1.5
			//1213
				goal_reached = true;
				//***********************************fusion
				cmd_flag=true;
				//***********************************fusion
				obs_lfw=0.4;
				move_base_msgs::MoveBaseActionResult goalresult;
	    			goalresult.header.stamp = ros::Time::now();
	    			goalresult.header.frame_id = "/map";
	    			goalresult.status.status = 3;
	    			result_waypt.publish(goalresult);
			}
		}
		else if(car2goal_dist < goalRadius){
            		goal_reached = true;
	    		smooth_counter = 20;
			//***********************************fusion
			cmd_flag=true;
			//***********************************fusion
		}
    	}
    	else{
		if(goal_reached == true && goal_reached_a == true){
			//***********************************fusion
			
			if(odom_goal_pos.z == 2.0){
				while(result_flag==false){;}
				cmd_flag=false;
				
				get_place();
				//sleep(5);
				/*
				move_base_msgs::MoveBaseActionResult goalresult;
	    			goalresult.header.stamp = ros::Time::now();
	    			goalresult.header.frame_id = "/map";
	    			goalresult.status.status = 3;
	    			result_.publish(goalresult);*/
				//odom_initial_flag.data=true;


				/*
				//sleep(1);
				//odom_initial_pub.publish(odom_initial_flag);
				//sleep(5);

				
		
				//for(int num=1;num>0;num--){
					
					//cmd_flag=false;
					//odom_initial_flag.data=true;
		
					//sleep(1);
					//odom_initial_pub.publish(odom_initial_flag);
					//if(num==1)
					//	sleep(1);			
					//else
					//	sleep(5);
			
					
			
					srv.request.type = 2;
					srv.request.x = tag_goal.pose.position.x;
					srv.request.y = tag_goal.pose.position.y;
					srv.request.theta = tag_goal.pose.position.z;
			
					//goal_1
					//srv.request.x = -0.8;
					//srv.request.y = 0.6;
					//goal_2
					//srv.request.x = -0.8;
					//srv.request.y = -0.65;
			
					//srv.request.x = 0.2;
					//srv.request.y = 0.0;
					
					//if(tag_goal.pose.position.z>1.0){
					//	srv.request.theta = 0.0; //1.57
				//		srv.request.y = 0.0; //0.5
					//}
				//	else{
				//		srv.request.theta = 0.0;
					}
					ROS_ERROR("%.2f",tag_goal.pose.position.z);
			
					if(client_.call(srv)) {
						//ROS_ERROR("call service success");
						srv.request.type = 0;
						while(srv.response.run_completed == false) {
							usleep(100000);
							ROS_ERROR("hello servie");
							client_.call(srv);
						}
					}
					else {
						//ROS_INFO("call service fail");
					}

				}
				sleep(5);
				ROS_ERROR("over");
		*/
	   		 }	
			
	    		goal_reached = false;
	    		goal_reached_a = false;
            		goal_received = false;
            		ROS_INFO("Goal Reached !");
			ROS_ERROR("%.2f",odom_goal_pos.z);
	    		
			cmd_flag=true;
			/*
			move_base_msgs::MoveBaseActionResult goalresult;
	   	 	goalresult.header.stamp = ros::Time::now();
	    		goalresult.header.frame_id = "/map";
	    		goalresult.status.status = 3;
	    		result_.publish(goalresult);*/
			//***********************************fusion
	    		//// "move_base/result"
			if(odom_goal_pos.z == 0.0){
	   		move_base_msgs::MoveBaseActionResult goalresult;
	    		goalresult.header.stamp = ros::Time::now();
	    		goalresult.header.frame_id = "/map";
	    		goalresult.status.status = 3;
	    		result_.publish(goalresult);
			}
        	}
    	}
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = odom.pose.pose;
    geometry_msgs::Twist carVel = odom.twist.twist;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    if(goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);
	//std::cout<<"x: "<<odom_goal_pos.x<<"   y: "<<odom_goal_pos.y<<std::endl;
        if(!goal_reached)
        {
            cmd_vel.angular.z = baseAngle + getSteeringAngle(eta)*Angle_gain *PI/180.0;

            /*Estimate Gas Input*/

                //double u = getGasInput(carVel.linear.x);
                //cmd_vel.linear.x = baseSpeed - u;

		//1209
		ROS_ERROR("path_size: %d",map_path.poses.size());
		if(map_path.poses.size()>0)
                	cmd_vel.linear.x = baseSpeed;
		else{
			cmd_vel.linear.x=0.0;
			if(path_recieved==false && goal_received ==true){
				if(odom_goal_pos.z == 2.0){}
				else{
					count++;
				
					if(count>=10){
						goal_pose.publish(odom_goal_revise);
						ROS_INFO("////////////////////////success");
						count=0;
					}
					//1212
					count_obs++;
					if(count_obs>=10){
						obstacle_flag.data=true;
						obstacle_clear.publish(obstacle_flag);
						ROS_INFO("////////////////////////success_obs");
						count_obs=0;
						obstacle_flag.data=false;
					}
				}
				//1212
			}		
		}	
		//1209

		if(odom_goal_pos.z == 0.0){
			if(getCar2GoalDist()<0.5)
				cmd_vel.linear.x = baseSpeed * getCar2GoalDist()/0.5;
		}
                //ROS_INFO("\nGas = %.2f\nSteering angle = %.2f",cmd_vel.linear.x,cmd_vel.angular.z);
		//***********************************fusion
		else if(odom_goal_pos.z == 2.0){
			if(getCar2GoalDist()<0.5)	
				cmd_vel.linear.x = baseSpeed * getCar2GoalDist()/0.5;
		}
		//***********************************fusion
		if(cmd_vel.angular.z >= 40.0 / 180.0 * PI){
			cmd_vel.angular.z = 40.0 / 180.0 * PI;
			cmd_vel.linear.x = 0;
	    	}
	    	else if(cmd_vel.angular.z <= -40.0 / 180.0 * PI){
			cmd_vel.angular.z = -40.0 / 180.0 * PI;
			cmd_vel.linear.x = 0;
            	}

        }
	else if(!goal_reached_a){
		if(smooth_counter > 0){
			smooth_counter--;
		}
		else{
			ROS_INFO("CIRCLEEEEEEEEEEEEEE");
			cmd_vel.angular.z = getStandingAngle(getYawFromPose(carPose));
			if(cmd_vel.angular.z == 0.0)
				goal_reached_a = true;
		}
	}
	else
	        //cmd_vel.linear.x = 0.1;
		ROS_INFO("Gas = XXX	Steering angle = XXX");
    }

    if(odom_goal_pos.z == 1.0)
	cmd_vel.linear.x = baseSpeed*0.8;

    //***********************************fusion
    if(cmd_flag){
    	cmd_vel = SafeVelocity(cmd_vel);

    	pub_.publish(cmd_vel);
    }
    //***********************************fusion
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "L1Controller_v2_0");
    L1Controller controller;
    ros::spin();
    return 0;
}
