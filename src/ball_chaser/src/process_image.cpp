#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"

ros::ServiceClient drive_client;
void drive_bot(ball_chaser::DriveToTarget& drv_srv){    
    if (!drive_client.call(drv_srv)){
        ROS_ERROR_STREAM("Failed to call service: " + drive_client.getService());
    }
}

void cam_sub_callback(const sensor_msgs::Image& img){
    bool ball_found{false};
    std::vector<int> pixel_info{0,0,0,0};
    ball_chaser::DriveToTarget drive_bot_srv;
    for (int i = 0 ; i <= img.height * img.step ; i += 3){
        pixel_info[0] =  img.data[i];
        pixel_info[1] =  img.data[i + 1];
        pixel_info[2] =  img.data[i + 2];
        pixel_info[3] =  i % (img.width * 3) / 3;

        if (pixel_info[0] == 255 && pixel_info[1] == 255  && pixel_info[2] == 255 ){
            if (pixel_info[3] >=0 && pixel_info[3] < img.width/3){
                drive_bot_srv.request.linear_x  = 0.0;
                drive_bot_srv.request.angular_z  = 0.75;
            }
            else if (pixel_info[3] >= img.width/3 && pixel_info[3] < img.width/3*2)
            {
                drive_bot_srv.request.linear_x  = 0.95;
                drive_bot_srv.request.angular_z  = 0.0;
            }
            else if (pixel_info[3] >= img.width/3*2 && pixel_info[3] <= img.width)
            {
                drive_bot_srv.request.linear_x  = 0.0;
                drive_bot_srv.request.angular_z  = -0.75;
            }
            else
            {
                drive_bot_srv.request.linear_x  = 0.0;
                drive_bot_srv.request.angular_z  = 0.0;
            }
            // ROS_INFO_STREAM("I can see the ball. Image width is " << img.width << ", and pixel posiiton is " << pixel_info[3]);         
            drive_bot(drive_bot_srv);
            ball_found = true;
            return;
            //break because no more processing is needed this iteration
        }
        else{
            //per pexil, do nothing
        } 
    }
    if(!ball_found){
        // if ball not found after processing the whole image
        drive_bot_srv.request.linear_x  = 0.0;
        drive_bot_srv.request.angular_z  = 0.0;
        drive_bot(drive_bot_srv);
        return;
    }
    
}
int main(int argc, char** argv){
    ros::init(argc, argv, "process_image");
    ros::NodeHandle nh;

    ros::Subscriber cam_sub = nh.subscribe("/camera/rgb/image_raw", 5, cam_sub_callback);
    drive_client = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    
    ros::spin();
}