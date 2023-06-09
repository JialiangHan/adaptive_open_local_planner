#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include "struct_defs.h"
#include "matrix.h"
namespace adaptive_open_local_planner
{
    class VisualizationHelpers
    {
    public:
        VisualizationHelpers(){};
        virtual ~VisualizationHelpers(){};

        static void createCurrentPoseMarker(const VehicleState &current_state_in_map_frame, geometry_msgs::PoseStamped &pose);
        static void createCarFootprintMarker(const std::vector<Waypoint> &car_footprint, visualization_msgs::Marker &car_footprint_marker);
        static void createSafetyBoxMarker(const std::vector<Waypoint> &safety_box, visualization_msgs::Marker &safety_box_marker);
        static void createBoxObstacleMarker(const BoxObstacle &b, visualization_msgs::Marker &box_obstacle_marker);

        static void createExtractedPathMarker(const std::vector<Waypoint> &extracted_path, nav_msgs::Path &path);
        static void createRollOutsMarker(const std::vector<std::vector<Waypoint>> &roll_outs, visualization_msgs::MarkerArray &roll_out_marker_array);
        static void createWeightedRollOutsMarker(const std::vector<std::vector<Waypoint>> &roll_outs, const std::vector<PathCost> &path_costs,
                                                 const int &best_index, visualization_msgs::MarkerArray &weighted_roll_out_marker_array);
    };
}; // end namespace adaptive_open_local_planner