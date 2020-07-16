#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <string>

ros::Publisher vel_pub;


bool cmd_rbt_callback(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
    geometry_msgs::Twist cmd_twist;
    
    while(ros::ok()){
        // cmd_twist.linear.x = ;
        // cmd_twist.angular.z = 0.0;
        cmd_twist.linear.x = req.linear_x;
        cmd_twist.angular.z = req.angular_z;

        vel_pub.publish(cmd_twist);
        // ROS_INFO_STREAM("Published Twsit with values linear: " << cmd_twist.linear", and angular: " << cmd_twist.angular <<" to the topic" << vel_pub.getTopic);
        res.msg_feedback = "Published Twsit with values linear: " + std::to_string(cmd_twist.linear.x) + ", and angular: " + std::to_string(cmd_twist.angular.z) + " to the topic " + vel_pub.getTopic();
        return true;
    }
}
int main(int argc, char** argv){
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceServer move_bot_server = nh.advertiseService("/ball_chaser/command_robot", cmd_rbt_callback);
    ROS_INFO_STREAM("Node " + ros::this_node::getName() + " is ready to drive the robot with service");

    ros::spin();
    return 0;
}
