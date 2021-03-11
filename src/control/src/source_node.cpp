#include <ros/ros.h>
#include <std_msgs/String.h>

#include "source_node.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "control_node");
   ros::NodeHandle n("~");

   UserDriver drv(n);

   std::thread publish(&UserDriver::input_loop, &drv);
   std::thread subscribe(UserDriver::rosLoop);

   publish.join();
   subscribe.join();

   return 0;
}