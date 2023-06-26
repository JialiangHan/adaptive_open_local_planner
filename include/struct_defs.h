#pragma once

#include <vector>
namespace adaptive_open_local_planner
{
    struct Waypoint
    {
        double x;
        double y;
        double heading;
        double left_width;
        double right_width;
        double speed = 0;
        double angular_speed = 0;
        double cost = 0;
        double time_cost = 0;
        bool operator==(const Waypoint &rhs) const
        {
            return (x == rhs.x) && (y == rhs.y) && (heading == rhs.heading); // or another approach as above
        };
    };

    typedef struct
    {
        double x;
        double y;
        double heading;
        double width;
        double length;
    } BoxObstacle;

    typedef struct
    {
        double x;
        double y;
        double yaw;
        double speed;
        double steer;
    } VehicleState;

    typedef struct
    {
        int front_index;
        int back_index;
        double perp_distance;
        double to_front_distance;
        double from_back_distance;
        double angle_diff;
        Waypoint perp_point;
    } RelativeInfo;

    typedef struct
    {
        int index;
        int relative_index;
        double closest_obj_velocity;
        double distance_from_center;
        double priority_cost;    // 0 to 1
        double transition_cost;  // 0 to 1
        double closest_obj_cost; // 0 to 1
        double cost;
        double closest_obj_distance;

        int lane_index;
        double lane_change_cost;
        double lateral_cost;
        double longitudinal_cost;
        double curvature_cost;
        bool bBlocked;
        std::vector<std::pair<int, double>> lateral_costs;
    } PathCost;

    typedef struct
    {
        double x;
        double y;
        double vx;
        double vy;
        double radius;
        double true_radius;
    } CircleObstacle;
};