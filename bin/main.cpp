#include <ros/ros.h>

#include "Camera/Camera.h"
#include "QRDecoder.h"

int main( int argc, char* argv[] )
{
   Camera d;

   ros::init(argc, argv, "example_node");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 10, imageCallback);
   ros::Rate loop_rate(50);

   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }

    return 0;
}
