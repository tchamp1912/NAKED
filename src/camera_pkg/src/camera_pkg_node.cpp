#include <ros/ros.h>
#include <std_msgs/String.h>

#include "Camera.h"
#include "QRDecode.h"

int main( int argc, char* argv[] )
{
   Camera cam(640, 480);
   cam.registerCallback(&detectAndDecode);

   ros::init(argc, argv, "camera_pkg_node");
   ros::NodeHandle n("~");
   ros::Subscriber img_sub = n.subscribe("/camera/rgb/image_raw", 10, &Camera::imageCallback, &cam);
   ros::Publisher qr_pub = n.advertise<std_msgs::String>("qr_code", 10);
   ros::Rate loop_rate(50);

   std::string data;
   std_msgs::String msg;
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
      data = cam.getData();
      msg.data = data;
      qr_pub.publish(msg);
   }

    return 0;
}
