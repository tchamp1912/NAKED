#include <ros/ros.h>

#include "Camera.h"
#include "QRDecode.h"

int main( int argc, char* argv[] )
{
   Camera cam(640, 480);
   cam.registerCallback(&detectAndDecode);

   ros::init(argc, argv, "camera_pkg_node");
   ros::NodeHandle n("~");
   ros::Subscriber img_sub = n.subscribe("/camera/rgb/image_raw", 10, &Camera::imageCallback, &cam);
   ros::Publisher qr_pub = n.advertise("qr_code", 10);
   ros::Rate loop_rate(50);

   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
      qr_pub.publish(std_msgs::String(cam.getData()));
   }

    return 0;
}
