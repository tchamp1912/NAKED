#include <ros/ros.h>
#include <std_msgs/String.h>

#include "control_node.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "control_node");
   ros::NodeHandle n;

   UserDriver drv(n);

   std::thread publish(&UserDriver::input_loop, &drv, std::ref(n));
   std::thread subscribe(UserDriver::rosLoop);

   publish.join();
   subscribe.join();

   return 0;
}