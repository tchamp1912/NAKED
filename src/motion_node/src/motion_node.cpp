#include <ros/ros.h>
#include "motion_node.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle n("~");
    Motion m_node(n);

    m_node.loop(n);

    return 0;
}