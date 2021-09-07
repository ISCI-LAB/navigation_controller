#include "ros/ros.h"
#include <stdio.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>


class vslam_localization{
public:
  vslam_localization(tf::TransformListener& tf);

private:

  ros::NodeHandle nh_;
  ros::Subscriber camera_pose_sub;

  tf::TransformListener& tf_; 
  tf::StampedTransform base2cam;
  tf::StampedTransform cam2base;

  Eigen::Matrix4f SlamCamPose_Matrix;
  Eigen::Matrix4f WorldCamPose_Matrix;
  Eigen::Matrix4f WorldBasePose_Matrix;
  Eigen::Matrix4f Base2Cam_Matrix;
  Eigen::Matrix4f Cam2Base_Matrix;

  double euler_x, euler_y, euler_z;

  void camera_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

vslam_localization::vslam_localization(tf::TransformListener& tf) : tf_(tf)
{
  camera_pose_sub = nh_.subscribe("/orb_slam2_rgbd/slam/camera_pose", 1, &vslam_localization::camera_pose_callback, this);
}

void vslam_localization::camera_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)

{
   static tf::TransformBroadcaster br;
   tf::Transform transform;

  //Read Camera Pose from slam
   //tf::Matrix3x3 slam_cam_pose_ori(msg->pose.orientation);
   Eigen::Quaterniond slam_cam_pose_q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
   Eigen::Matrix3d slam_cam_pose_ori = slam_cam_pose_q.toRotationMatrix();

   for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      SlamCamPose_Matrix(i, j) = slam_cam_pose_ori(i, j);

   SlamCamPose_Matrix(0, 3) = msg->pose.position.x;
   SlamCamPose_Matrix(1, 3) = msg->pose.position.y;
   SlamCamPose_Matrix(2, 3) = msg->pose.position.z;
   SlamCamPose_Matrix(3, 3) = 1;
   SlamCamPose_Matrix(3, 0) = 0;
   SlamCamPose_Matrix(3, 1) = 0;
   SlamCamPose_Matrix(3, 2) = 0;

   // Camera to World transform
   try{
     tf_.waitForTransform("/camera_color_optical_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
     tf_.lookupTransform("/camera_color_optical_frame", "/base_link", ros::Time(0), base2cam);
   }
   catch (tf::TransformException &ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
     //continue;
   }

   //tf::Matrix3x3 base_to_cam_ori(base2cam.getRotation());
   Eigen::Quaterniond base_to_cam_ori_q(base2cam.getRotation().w(), base2cam.getRotation().x(), base2cam.getRotation().y(), base2cam.getRotation().z());
   Eigen::Matrix3d base_to_cam_ori = base_to_cam_ori_q.toRotationMatrix();

   for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      Base2Cam_Matrix(i, j) = base_to_cam_ori(i, j);

   Base2Cam_Matrix(0, 3) = base2cam.getOrigin().x();
   Base2Cam_Matrix(1, 3) = base2cam.getOrigin().y();
   Base2Cam_Matrix(2, 3) = base2cam.getOrigin().z();
   Base2Cam_Matrix(3, 3) = 1;
   Base2Cam_Matrix(3, 0) = 0;
   Base2Cam_Matrix(3, 1) = 0;
   Base2Cam_Matrix(3, 2) = 0;

   //WorldCamPose_Matrix = Base2Cam_Matrix * SlamCamPose_Matrix;
   WorldCamPose_Matrix = SlamCamPose_Matrix * Base2Cam_Matrix;

   // Base to World transform
   try{
     tf_.waitForTransform("/base_link", "/camera_color_optical_frame", ros::Time(0), ros::Duration(3.0));
     tf_.lookupTransform("/base_link", "/camera_color_optical_frame", ros::Time(0), cam2base);
   }
   catch (tf::TransformException &ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
     //continue;
   }

   //tf::Matrix3x3 cam_to_base_ori(cam2base.getRotation());
   Eigen::Quaterniond cam_to_base_ori_q(cam2base.getRotation().w(), cam2base.getRotation().x(), cam2base.getRotation().y(), cam2base.getRotation().z());
   Eigen::Matrix3d cam_to_base_ori = cam_to_base_ori_q.toRotationMatrix();

   for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      Cam2Base_Matrix(i, j) = cam_to_base_ori(i, j);

   Cam2Base_Matrix(0, 3) = cam2base.getOrigin().x();
   Cam2Base_Matrix(1, 3) = cam2base.getOrigin().y();
   Cam2Base_Matrix(2, 3) = cam2base.getOrigin().z();
   Cam2Base_Matrix(3, 3) = 1;
   Cam2Base_Matrix(3, 0) = 0;
   Cam2Base_Matrix(3, 1) = 0;
   Cam2Base_Matrix(3, 2) = 0;

   //WorldBasePose_Matrix = WorldCamPose_Matrix * Cam2Base_Matrix;
   WorldBasePose_Matrix = Cam2Base_Matrix * WorldCamPose_Matrix;

   if(WorldBasePose_Matrix(2, 0) != 1 && WorldBasePose_Matrix(2, 0)!= -1){
      euler_y = -asin(WorldBasePose_Matrix(2, 0));
      euler_x = atan2( (WorldBasePose_Matrix(2, 1) / cos(euler_y)) , (WorldBasePose_Matrix(2, 2) / cos(euler_y)) );
      euler_z = atan2( WorldBasePose_Matrix(1, 0) , WorldBasePose_Matrix(0, 0) );
   }
   else
      ROS_ERROR("Euler angle error!!!");

   //ROS_WARN("Robot State : x = %lf, y = %lf, z = %lf, rx = %lf, ry = %lf, rz = %lf", WorldBasePose_Matrix(0, 3), WorldBasePose_Matrix(1, 3), WorldBasePose_Matrix(2, 3), euler_x, euler_y, euler_z);

   transform.setOrigin( tf::Vector3(WorldBasePose_Matrix(0, 3), WorldBasePose_Matrix(1, 3), WorldBasePose_Matrix(2, 3)) );
   tf::Quaternion q;
   q.setRPY(euler_x, euler_y, euler_z);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/vslam_map", "/base_link"));
}

int main(int argc, char **argv)

{
  ros::init(argc, argv, "vslam_state_pub");

  tf::TransformListener tf(ros::Duration(10));
  vslam_localization VL(tf);
  // ros::NodeHandle n;
  // ros::Subscriber camera_pose_sub = n.subscribe("/orb_slam2_rgbd/slam/camera_pose", 1000, camera_pose_callback);
  ros::spin();

  return 0;
}
