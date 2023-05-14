

#include "parameter_manager.h"

namespace adaptive_open_local_planner
{

  void ParameterManager::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
  {

    nh.getParam("odom_topic", odom_topic);
    nh.getParam("map_frame", map_frame);
    nh.getParam("obstacles_topic", obstacles_topic);
    nh.getParam("extracted_path_rviz_topic", extracted_path_rviz_topic);
    nh.getParam("current_pose_rviz_topic", current_pose_rviz_topic);
    nh.getParam("roll_outs_rviz_topic", roll_outs_rviz_topic);
    nh.getParam("weighted_trajectories_rviz_topic", weighted_trajectories_rviz_topic);
    nh.getParam("safety_box_rviz_topic", safety_box_rviz_topic);
    nh.getParam("car_footprint_rviz_topic", car_footprint_rviz_topic);
    nh.getParam("box_obstacle_rviz_topic", box_obstacle_rviz_topic);
    nh.getParam("cmd_vel_topic", cmd_vel_topic);
    // nh.getParam("planning_frequency", planning_frequency);
    nh.getParam("max_speed", max_speed);
    nh.getParam("max_local_plan_distance", max_local_plan_distance);
    nh.getParam("path_density", path_density);
    nh.getParam("roll_outs_number", roll_outs_number);
    nh.getParam("sampling_tip_margin", sampling_tip_margin);
    nh.getParam("sampling_out_margin", sampling_out_margin);
    nh.getParam("roll_out_density", roll_out_density);
    nh.getParam("roll_in_speed_factor", roll_in_speed_factor);
    nh.getParam("roll_in_margin", roll_in_margin);
    nh.getParam("lane_change_speed_factor", lane_change_speed_factor);
    nh.getParam("horizon_distance", horizon_distance);
    nh.getParam("horizontal_safety_distance", horizontal_safety_distance);
    nh.getParam("vertical_safety_distance", vertical_safety_distance);
    nh.getParam("max_steer_angle", max_steer_angle);
    nh.getParam("min_speed", min_speed);
    nh.getParam("lateral_skip_distance", lateral_skip_distance);
    nh.getParam("min_following_distance", min_following_distance);
    nh.getParam("max_following_distance", max_following_distance);
    nh.getParam("min_distance_to_avoid", min_distance_to_avoid);
    nh.getParam("vehicle_width", vehicle_width);
    nh.getParam("vehicle_length", vehicle_length);
    nh.getParam("wheelbase_length", wheelbase_length);
    nh.getParam("turning_radius", turning_radius);
    nh.getParam("safety_radius", safety_radius);
    nh.getParam("smooth_data_weight", smooth_data_weight);
    nh.getParam("smooth_weight", smooth_weight);
    nh.getParam("smooth_tolerance", smooth_tolerance);
    nh.getParam("priority_weight", priority_weight);
    nh.getParam("transition_weight", transition_weight);
    nh.getParam("lat_weight", lat_weight);
    nh.getParam("long_weight", long_weight);
    nh.getParam("collision_weight", collision_weight);
    nh.getParam("curvature_weight", curvature_weight);

    nh.getParam("evaluate_path", evaluate_path);

    nh.getParam("weighting", weighting);
    nh.getParam("personal_learning_rate", personal_learning_rate);
    nh.getParam("global_learning_rate", global_learning_rate);
    nh.getParam("cost_difference_boundary", cost_difference_boundary);
    nh.getParam("max_interation", max_interation);

    nh.getParam("max_linear_velocity", max_linear_velocity);
    nh.getParam("min_linear_velocity", min_linear_velocity);
    nh.getParam("max_angular_acceleration", max_angular_acceleration);
    nh.getParam("min_angular_acceleration", min_angular_acceleration);
  }

} // namespace adaptive_open_local_planner
