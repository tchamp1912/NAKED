#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <map>
#include <iostream>


class Motion {
public:
  Motion(ros::NodeHandle nh) {
    motion_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/wheel", 1, &Motion::odometryCallback, this);
    control_sub = nh.subscribe<std_msgs::String>("control_move", 1, &Motion::movementCallback, this);
  };	

  enum Movement {
    FORWARD = 'i',
    RIGHT = 'l',
    LEFT = 'j',
    BACK = 'k',
    STOP = ','
  };

  void odometryCallback(nav_msgs::Odometry odom) {
    //std::cout << odom.pose.pose.position.x << std::endl;
    //std::cout << odom.pose.pose.position.y << std::endl;
    //std::cout << odom.pose.pose.position.z << std::endl;

    //std::cout << odom.pose.orientation << std::endl;
  }

  void movementCallback(std_msgs::String cmd) 
  {
    switch(cmd.data[0]) {
    case FORWARD:
      movement.linear.x = speed * 1;
      movement.linear.y = speed * 0;
      movement.linear.z = speed * 0;
      movement.angular.x = 0;
      movement.angular.y = 0;
      movement.angular.z = turn * 0;
      break;
    case RIGHT:
      movement.linear.x = speed * 0;
      movement.linear.y = speed * 0;
      movement.linear.z = speed * 0;
      movement.angular.x = 0;
      movement.angular.y = 0;
      movement.angular.z = turn * -1;
      break;
    case LEFT:
      movement.linear.x = speed * 0;
      movement.linear.y = speed * 0;
      movement.linear.z = speed * 0;
      movement.angular.x = 0;
      movement.angular.y = 0;
      movement.angular.z = turn * 1;
      break;
    case BACK:
      movement.linear.x = speed * -1;
      movement.linear.y = speed * 0;
      movement.linear.z = speed * 0;
      movement.angular.x = 0;
      movement.angular.y = 0;
      movement.angular.z = turn * 0;
    case STOP:
      movement.linear.x = speed * 0;
      movement.linear.y = speed * 0;
      movement.linear.z = speed * 0;
      movement.angular.x = 0;
      movement.angular.y = 0;
      movement.angular.z = turn * 0;
      break;
    }

    motion_pub.publish(movement); 
  };

  void loop(ros::NodeHandle n) {
    ros::Rate loop_rate(10);
    while(n.ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
private:
  // Motion specific parameters
  geometry_msgs::Twist movement;
  static const float speed = 0.5;
  static const float turn = 1.0;

  // ROS componenets
  ros::Publisher motion_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber control_sub; 
};
