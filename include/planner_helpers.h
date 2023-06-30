#pragma once

#include <ros/ros.h>
#include <vector>
#include <tf/tf.h>
#include "struct_defs.h"
#include "matrix.h"
#include "polygon.h"
#include "visualization_helpers.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#define _USE_MATH_DEFINES
#define RAD2DEG 180.0 / M_PI
#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x *v.x + v.y * v.y)
namespace adaptive_open_local_planner
{
    class PlannerHelpers
    {
    public:
        PlannerHelpers(){};
        virtual ~PlannerHelpers(){};

        static int getClosestNextWaypointIndex(const std::vector<Waypoint> &path, const Waypoint &current_pos);
        static double angleBetweenTwoAnglesPositive(const double &a1, const double &a2);
        static double fixNegativeAngle(const double &a);
        /**
         * @brief
         *
         * @param path
         * @param path_density
         */
        static void fixPathDensity(std::vector<Waypoint> &path, const double &path_density);
        static void smoothPath(std::vector<Waypoint> &path, const double &smooth_tolerance, const double &smooth_data_weight, const double &smooth_weight);
        static void calculateAngleAndCost(std::vector<Waypoint> &path, const double &prev_cost_);
        static bool getRelativeInfo(const std::vector<Waypoint> &path, const Waypoint &current_pos, RelativeInfo &info);
        static void predictConstantTimeCostForTrajectory(std::vector<Waypoint> &path, const VehicleState &current_state_in_map_frame);
        static double getExactDistanceOnTrajectory(const std::vector<Waypoint> &trajectory, const RelativeInfo &p1, const RelativeInfo &p2);
        static double checkTrajectoryForCollision(const std::vector<Waypoint> &trajectory, const std::vector<CircleObstacle> &circle_obstacles, const std::vector<BoxObstacle> &box_obstacles,
                                                  const double &safety_radius, const double &vehicle_width, const double &vehicle_length, const double &wheelbase_length,
                                                  const double &horizontal_safety_distance, const double &vertical_safety_distance);
        static void calculateTransitionCosts(std::vector<PathCost> &trajectory_costs, const int &curr_trajectory_index, const double &roll_out_density);
        static void calculateLateralAndLongitudinalCostsStatic(std::vector<PathCost> &trajectory_costs, const std::vector<std::vector<Waypoint>> &roll_outs,
                                                               const std::vector<Waypoint> &extracted_path, std::vector<Waypoint> &contour_points,
                                                               const VehicleState &current_state_in_map_frame, visualization_msgs::Marker &car_footprint_marker,
                                                               visualization_msgs::Marker &safety_box_marker,
                                                               const double &vehicle_length, const double &vehicle_width,
                                                               const double &wheelbase_length, const double &horizontal_safety_distance,
                                                               const double &vertical_safety_distance, const double &max_steer_angle,
                                                               const double &min_following_distance, const double &lateral_skip_distance);
        static void calculateCurvatureCosts(std::vector<PathCost> &trajectory_costs, const std::vector<std::vector<Waypoint>> &roll_outs);
        static void normalizeCosts(std::vector<PathCost> &trajectory_costs, const double &priority_weight,
                                   const double &transition_weight, const double &lat_weight, const double &long_weight, const double &curvature_weight);

        static void convert(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan, std::vector<Waypoint> &path);

        static void convert(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan, nav_msgs::Path &path);

        static std::vector<Waypoint> convert(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        static Waypoint convert(const geometry_msgs::PoseStamped &pose);

        static Eigen::Vector2f convert(const Waypoint &way_point);

        static std::vector<Eigen::Vector3f> convert(const nav_msgs::Path::ConstPtr &path);

        static float CalculateCurvature(const Eigen::Vector2f &pre, const Eigen::Vector2f &current, const Eigen::Vector2f &succ);

        static float CalculateCurvature(const Waypoint &pre, const Waypoint &current);

        static float Clamp(const float &number, const float &upper_bound,
                           const float &lower_bound);

        static std::vector<Waypoint> extractVector(const std::vector<Waypoint> &base, int start_index, int end_index);

        static float getDistance(const std::vector<Waypoint> &local_path);

        static float getDistance(const Waypoint &prev, const Waypoint &current);

        static float sumVector(const std::vector<float>& vector);

        static std::vector<float> getFirstVec(const std::vector<std::pair<float, float>> &pair_vec);

        static std::vector<float> getSecondVec(const std::vector<std::pair<float, float>> &pair_vec);
    };
};