#include <ros/ros.h>
#include "adaptive_open_local_planner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "adaptive_open_local_planner");
    AdaptiveOpenLocalPlanner junior_local_planner_obj;
    ros::spin();
    return 0;
}