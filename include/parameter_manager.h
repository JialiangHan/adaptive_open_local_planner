#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace adaptive_open_local_planner
{

  /**
   * @class ParameterManager
   * @brief Config class for the teb_local_planner and its components.
   */
  class ParameterManager
  {
  public:
    std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
    std::string map_frame;  //!< Global planning frame

    std::string obstacles_topic;

    std::string extracted_path_rviz_topic;
    std::string current_pose_rviz_topic;
    std::string roll_outs_rviz_topic;
    std::string weighted_trajectories_rviz_topic;
    std::string safety_box_rviz_topic;
    std::string car_footprint_rviz_topic;
    std::string box_obstacle_rviz_topic;
    std::string cmd_vel_topic;

    double planning_frequency;

    // Parameters
    double max_speed;               // max speed that planner should not exceed
    double max_local_plan_distance; // length of local trajectory roll outs
    double path_density;            // distance between waypoints of local trajectory
    int roll_outs_number;           // number of roll outs not including the center trajectory (this number should be even)
    double sampling_tip_margin;     // length of car tip margin
    double sampling_out_margin;     // length of roll in margin (??)
    double roll_out_density;        // distance between adjacent trajectories
    double roll_in_speed_factor;
    double roll_in_margin;
    double lane_change_speed_factor;
    double horizon_distance;

    double horizontal_safety_distance;
    double vertical_safety_distance;
    double max_steer_angle;
    double min_speed;
    double lateral_skip_distance;

    double min_following_distance; // distance threshold for exiting following behaviour
    double max_following_distance; // distance threshold for entering following behaviour
    double min_distance_to_avoid;  // distance threshold for obstacle avoidance behaviour

    double vehicle_width;
    double vehicle_length;
    double wheelbase_length;
    double turning_radius;
    double safety_radius;

    // Smoothing Weights
    double smooth_data_weight;
    double smooth_weight;
    double smooth_tolerance;

    double priority_weight;
    double transition_weight;
    double lat_weight;
    double long_weight;
    double collision_weight;
    double curvature_weight;
    double max_global_plan_lookahead_dist;
    double global_plan_prune_distance;

    double xy_goal_tolerance;
    double yaw_goal_tolerance;
    double transform_tolerance;

    bool evaluate_path;
    // PSO parameter
    float weighting;
    float personal_learning_rate;
    float global_learning_rate;
    float cost_difference_boundary;
    float max_interation;
    // speed limit
    float max_linear_velocity;
    float min_linear_velocity;
    float max_angular_acceleration;
    float min_angular_acceleration;
    // factor to control how much sub path you want to divide, it is K in the paper, should be larger than zero, smaller than one.
    float path_divide_factor;
    int number_of_particle;
    ParameterManager()
    {
      odom_topic = "odom";
      map_frame = "odom";

      obstacles_topic = "obstacles";

      extracted_path_rviz_topic = "extracted_path_rviz";
      current_pose_rviz_topic = "current_pose_rviz";
      roll_outs_rviz_topic = "roll_outs_rviz";
      weighted_trajectories_rviz_topic = "weighted_trajectories_rviz";
      safety_box_rviz_topic = "safety_box_rviz";
      car_footprint_rviz_topic = "car_footprint_rviz";
      box_obstacle_rviz_topic = "box_obstacle_rviz";
      cmd_vel_topic = "cmd_vel";

      planning_frequency = 1;

      // Parameters
      max_speed = 2;                // max speed that planner should not exceed
      max_local_plan_distance = 12; // length of local trajectory roll outs
      path_density = 0.3;           // distance between waypoints of local trajectory
      roll_outs_number = 8;         // number of roll outs not including the center trajectory (this number should be even)
      sampling_tip_margin = 1.2;    // length of car tip margin
      sampling_out_margin = 5;      // length of roll in margin (??)
      roll_out_density = 0.3;       // distance between adjacent trajectories
      roll_in_speed_factor = 0.2;
      roll_in_margin = 6;
      lane_change_speed_factor = 0.5;
      horizon_distance = 15;

      horizontal_safety_distance = 0.3;
      vertical_safety_distance = 0.3;
      max_steer_angle = 0.5;
      min_speed = 0.2;
      lateral_skip_distance = 10;

      min_following_distance = 5;  // distance threshold for exiting following behaviour
      max_following_distance = 10; // distance threshold for entering following behaviour
      min_distance_to_avoid = 8;   // distance threshold for obstacle avoidance behaviour

      vehicle_width = 1.25;
      vehicle_length = 1.8;
      wheelbase_length = 1.006;
      turning_radius = 1.5;
      safety_radius = 1.5;

      // Smoothing Weights
      smooth_data_weight = 0.45;
      smooth_weight = 0.4;
      smooth_tolerance = 0.1;

      priority_weight = 0.9;
      transition_weight = 0.9;
      lat_weight = 1.5;
      long_weight = 1;
      collision_weight = 2;
      curvature_weight = 1.2;

      max_global_plan_lookahead_dist = 3.0;
      global_plan_prune_distance = 1;

      xy_goal_tolerance = 0.2;
      yaw_goal_tolerance = 0.1;
      transform_tolerance = 0.6;

      evaluate_path = true;

      // PSO parameter
      weighting = 0.1;
      personal_learning_rate = 0.1;
      global_learning_rate = 0.1;
      cost_difference_boundary = 1;
      max_interation = 10000;
      // speed limit
      max_linear_velocity = 5;
      min_linear_velocity = 0;
      max_angular_acceleration = 10;
      min_angular_acceleration = -10;

      path_divide_factor = 0.3;

      number_of_particle = 100;
    }

    /**
     * @brief Load parameters from the ros param server.
     * @param nh const reference to the local ros::NodeHandle
     */
    void loadRosParamFromNodeHandle(const ros::NodeHandle &nh);
  };

} // namespace adaptive_open_local_planner
