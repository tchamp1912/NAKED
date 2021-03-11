#include <ros/ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <map>
#include <iostream>

enum Movement {

};

class Motion {
public:
  Motion(ros::NodeHandle nh) :
     {
        motion_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
        odom_sub = nh.subscriber<nav_msgs::Odometry>("odom", 1, &Motion::odometryCallback, this);
	      control_sub = nh.subscriber<std_msgs::String>("control_move", 1, &Motion::movementCallback, this);

        rate = ros::Rate(2);
        float speed = .5;
        float turn = 1.0;
      };	

  enum Movement {
    FORWARD = 'i',
    RIGHT = 'l',
    LEFT = 'j',
    BACK = 'k''
  };

  bool odometryCallback(nav_msgs::Odometry odom) {
    std::cout << odom.pose.point << std::endl;
    std::cout << odom.pose.orientation << std::endl;
  }

  bool movementCallback(std_msgs::String cmd) 
  {
    switch(cmd.data) {
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
  };

  void loop(ros::NodeHandle n) {
    while(n.ok()) {
      ros::SpinOnce();

      motion_pub.pub(movement); 
    }
  }
  
private:
  // Motion specific parameters
  geometry_msgs::Twist movement = geometry_msgs::Twist(0,0,0,0,0,0);

  // ROS componenets
  ros::Publisher<geometry_msgs::Twist> motion_pub;
  ros::Subscriber<nav_msgs::Odometry> odom_sub;
  ros::Subscriber<std_msgs::String> control_sub; 
};
