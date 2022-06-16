/**
 * @author Antonello Pastore
 * @date 15.06.2022
 * @brief ROS Node to manoeuvre the robot around an obstacle as soon as it is detected.
 **/
#include "ros/ros.h"                // ROS source header
#include "sensor_msgs/LaserScan.h"  // Message type for receiving scan data
#include <geometry_msgs/Twist.h>    // Message type for sending cmd_vel
#include <math.h>                   // For common mathematical operations

ros::Publisher vel_pub;     // Initialize the Publisher
ros::Subscriber scan_sub;   // Initialize the Subscriber

/**
 * @brief Callback function for the laser scan data
 * @param msg Laser scan data.
 */
void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  float range_ahead = scan_msg->ranges[0];
  // float range_ahead = scan_msg->ranges[msg.ranges.size() / 2];
}

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "edu_obstacle_bypass_node");

  // Create a ROS node handle
  ros::NodeHandle n;

  /** PUBLISHER
  * The advertise() function is how you tell ROS that you want to
  * publish on a given topic name: the generated twist-msg to move your robot.
  * Tell the master that we are going to be publishing every 10ms the message of type geometry_msgs/Twist on the topic /cmd_vel.
  */
  vel_pub = n.advertise<geometry_msgs::Twist>("/vel/teleop", 1);

  /** SUBSCRIBER
  * The subscribe() call is how you tell ROS that you want to receive messages
  * on a given topic.  Messages are passed to a callback function, here
  * called  scanCallback to determine the target angle.
  */
  scan_sub = n.subscribe("/front/scan", 1000, scanCallback);


  ros::Time state_change_time = ros::Time::now(); // Initialize the time of the last state change
  ros::Rate loop_rate(10);                        // Set the loop rate to 10 Hz

  float range_ahead = 0.0;                        // Initialize the range ahead to 0.0
  float min_range_ahead = 0.5;                    // Set the minimum range ahead in meters
  bool obstacle_detected = false;                 // Initialize the obstacle detected flag to false

  float min_spin_duration = 5.0;                  // Set the minimum spin duration in seconds
  float max_spin_duration = 10.0;                 // Set the maximum spin duration in seconds
  float spin_duration = 0.0;                      // Initialize the spin duration to 0.0 seconds
  int spin_direction = 1;                         // Initialize the spin direction to 1 (clockwise)

  float angular_velocity = 0.5;                   // Set the angular velocity in rad/s
  float linear_velocity = 0.5;                    // Set the linear velocity in 0.0 m/s

  while (ros::ok())
  {
    // Check if the obstacle is detected
    if (!obstacle_detected)
    {
      std::cout << "Driving forward. Range ahead [m]: " << range_ahead << std::endl;
      if (range_ahead < min_range_ahead || ros::Time::now() > state_change_time)
      {
        obstacle_detected = true;
        state_change_time = ros::Time::now() + ros::Duration(0.5);
      }      
    }
    else
    {
      std::cout << "Obstacle detected & turning. Range ahead [m]: " << range_ahead << std::endl;
      if (ros::Time::now() > state_change_time)
      {
        obstacle_detected = false;
        // Set the spin_duration to a random value between min_spin_duration and max_spin_duration [s]
        spin_duration = ((float)rand() / (float)RAND_MAX * (max_spin_duration - min_spin_duration)) + min_spin_duration;
        // Set the spin_direction to 1 (clockwise) or -1 (counter-clockwise)
        spin_direction = ((float)rand() / (float)RAND_MAX * 2.0) - 1.0;
        state_change_time = ros::Time::now() + ros::Duration(spin_duration);
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