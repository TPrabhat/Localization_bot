#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>

// Define a global client that can request services
ros::ServiceClient client;

bool obstacle_present = false;
bool closing_in = false;
float velocity, direction;


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    client.call(srv);

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /command_robot");
}

// This callback function continuously executes and reads the scan data
void scan_callback(const sensor_msgs::LaserScan scan)
{
    obstacle_present = false;
    for(int i = 0; i < sizeof(scan.ranges)/sizeof(scan.ranges[0]); i++)
    {
        if(scan.ranges[i] <= 2.0)
        {
	   ROS_INFO_STREAM("Obstacle ahead ");
           ROS_INFO_STREAM("Range: "+std::to_string(scan.ranges[i]));
           obstacle_present = true;
           break;
        }
    }

    if(obstacle_present)
       drive_robot(-2.0, 1.0);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    if(obstacle_present)
         return;

    closing_in = false;

     for (int height = 400; height < img.height; height++) 
      {
       for (int width = 0; width < img.step; width+=3) 
        {
       
       //   ROS_INFO_STREAM("Pixel: " + std::to_string(i) + " Pixel value: " + std::to_string(img.data[i]));
       if (img.data[width*height] == white_pixel & img.data[(width*height)+1] == white_pixel & img.data[(width*height)+2] == 255)
        {
          velocity = height * 0.002 - (abs(width - img.step/2.0)) * 0.0001;
          direction = (width - img.step/2.0) * 0.0004 - height * 0.0005;

          closing_in = true;
          ROS_INFO_STREAM("height : " + std::to_string(height) + " width : " + std::to_string(width));

          ROS_INFO_STREAM("velocity : " + std::to_string(velocity) + " direction : " + std::to_string(direction));
          break;          
        } 
      } 
      if(closing_in)
         break;
     }

    if(!closing_in)
      {
       ROS_INFO_STREAM("Searching ...");
       drive_robot(0.5, 0.3);
      }
    else
      {
       ROS_INFO_STREAM("Closing in ...");
       //ROS_INFO_STREAM("velocity : " + std::to_string(velocity) + " direction : " + std::to_string(direction));
       drive_robot(velocity, direction);
      }

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /scan to look for obstacle
    ros::Subscriber sub1 = n.subscribe("/scan", 10, scan_callback);

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function if there isnt an obstacle

    ros::Subscriber sub2 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
