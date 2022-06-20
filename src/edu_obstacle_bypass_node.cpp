/**
 * @author Antonello Pastore
 * @date 15.06.2022
 * @brief ROS Node to manoeuvre the robot around an obstacle as soon as it is detected.
 **/
#include "ros/ros.h"                // ROS source header
#include "sensor_msgs/LaserScan.h"  // Message type for receiving scan data
#include <geometry_msgs/Twist.h>    // Message type for sending cmd_vel
#include <math.h>                   // For common mathematical operations

class ScanListener 
{
  public:
    double range_ahead;
    double range_left;
    double range_right;

    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
};

void ScanListener::scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  range_ahead = scan_msg->ranges[0];                              // Range of the laser beam in front of the robot
  
  /*
  * If the robot is seeing itself while looking left or right, change the following factors as follows:
  * for left_index = decrease it to like 0.2 to 0.15
  * for right_index = increase it to like 0.8 to 0.85
  */
  int left_index = (int) round(scan_msg->ranges.size() * 0.25);   // Index of the leftmost range
  int right_index = (int) round(scan_msg->ranges.size() * 0.75);  // Index of the rightmost range
  
  range_left = scan_msg->ranges[left_index];                      // Range of the leftmost range
  range_right = scan_msg->ranges[right_index];                    // Range of the rightmost range
}

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "edu_obstacle_bypass_node");

  // Create a ROS node handle
  ros::NodeHandle n;

  // Create a ScanListener object
  ScanListener scan_listener;

  // Create a publisher for cmd_vel messages
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/vel/teleop", 1);

  // Create a subscriber for scan data
  ros::Subscriber scan_sub = n.subscribe("/front/scan", 1000, &ScanListener::scanCallback, &scan_listener);

  ros::Time state_change_time = ros::Time::now();                       // Initialize the time of the last state change
  ros::Rate loop_rate(10);                                              // Set the loop rate to 10 Hz

  scan_listener.range_ahead = 0.0;                                      // Initialize the range ahead to 0.0
  float min_range_ahead = 1.5;                                          // Set the minimum range ahead in meters
  float min_range_left = 0.8;                                           // Set the minimum range left in meters
  float min_range_right = 0.8;                                          // Set the maximum range right in meters
  bool obstacle_detected = false;                                       // Initialize the obstacle detected flag to false

  float angular_velocity = 0.5;                                         // Set the angular velocity in rad/s
  float linear_velocity = 0.5;                                          // Set the linear velocity in 0.0 m/s

  float min_spin_duration = 5.0       // Set the minimum spin duration in seconds
  float max_spin_duration = 10.0       // Set the maximum spin duration in seconds
  float spin_duration = 0.0;                                            // Initialize the spin duration to 0.0 seconds
  float spin_direction = 1.0;                                           // Initialize the spin direction to 1 (clockwise)

  while (ros::ok())
  {
    // Check if the obstacle is detected
    if (!obstacle_detected)
    {
      std::cout << "Driving forward. Range ahead [m]: " << scan_listener.range_ahead << std::endl;
      if (scan_listener.range_ahead < min_range_ahead || scan_listener.range_left < min_range_left || scan_listener.range_right < min_range_right)
      {
        obstacle_detected = true;
        // Set the spin_duration to a random value between min_spin_duration and max_spin_duration [s]
        spin_duration = ((float)rand() / (float)RAND_MAX * (max_spin_duration - min_spin_duration)) + min_spin_duration;
        state_change_time = ros::Time::now() + ros::Duration(spin_duration);
      }      
    }
    else
    {
      // Console log for debugging while testing the two newly considered scan ranges
      std::cout << "Obstacle detected & turning." << std::endl;
      std::cout << "Range ahead [m]: " << scan_listener.range_ahead << std::endl;
      std::cout << "Range left [m]: " << scan_listener.range_left << std::endl;
      std::cout << "Range right [m]: " << scan_listener.range_right << std::endl;
      if (ros::Time::now() > state_change_time)
      {
        obstacle_detected = false;
        state_change_time = ros::Time::now();
      }
    }
    
    geometry_msgs::Twist msg; // Create a message of type geometry_msgs/Twist to send cmd_vel data to the robot

    if (!obstacle_detected)
    {
      msg.linear.x = linear_velocity;
      msg.angular.z = 0.0;
    }
    else
    {
      msg.linear.x = 0.0;
      msg.angular.z = angular_velocity;
    }

    vel_pub.publish(msg); // Publish the message
    ros::spinOnce();      // Spin once to handle callbacks
    loop_rate.sleep();    // Sleep for the rest of the cycle
  }
}