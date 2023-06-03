/**
 * @file velocity_planner.h
 * @author Jialiang Han
 * @brief this file is to plan a velocity planner based on a local path, ref:A curvature-segmentation-based minimum time algorithm for autonomous vehicle velocity planning
 * @version 0.1
 * @date 2023-04-30
 *
 * @copyright Copyright (c) 2023
 *
 **/
#pragma once
#include "planner_helpers.h"
#include "pso.h"
namespace adaptive_open_local_planner
{
    class VelocityPlanner
    {
    public:
        VelocityPlanner(){};

        VelocityPlanner(const float &path_divide_factor,  const float &max_linear_velocity, const float &min_linear_velocity, const float &max_angular_acceleration, const float &min_angular_acceleration, const float &max_linear_acceleration, const float &min_linear_acceleration, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle);

        std::vector<float> planVelocity(const std::vector<Waypoint> &local_path,const float &current_speed);

    private:
        /**
         * @brief divide local path by curvature change
         *
         * @param local_path
         * @param divided_path
         */
        void dividePath(const std::vector<Waypoint> &local_path, std::vector<std::vector<Waypoint>> &divided_path);

        float findDeltaCurvature(const std::vector<Waypoint> &local_path);

        float findDeltaCurvature(const std::vector<Waypoint> &local_path, int start_index, int end_index);
        /**
         * @brief find max curvature of local path
         *
         * @param local_path
         * @param max_curvature
         */
        void findMaxCurvature(const std::vector<Waypoint> &local_path, float &max_curvature);

        void findMaxCurvature(const std::vector<Waypoint> &local_path, float &max_curvature, int start_index, int end_index);

        /**
         * @brief find min curvature of local path
         *
         * @param local_path
         * @param min_curvature
         */
        void findMinCurvature(const std::vector<Waypoint> &local_path, float &min_curvature);

        void findMinCurvature(const std::vector<Waypoint> &local_path, float &max_curvature, int start_index, int end_index);
        /**
         * @brief for every point on the local path, find its corresponding velocity limit both linear and angular
         *
         * @param local_path
         */
        void findVelocityBoundary(const std::vector<std::vector<Waypoint>> &divided_path);

    private:
        float path_divide_factor_;
        // PSO parameter
        float weighting_;
        float personal_learning_rate_;
        float global_learning_rate_;
        float cost_difference_boundary_;
        float max_interation_;
        int number_of_particle_;
        // speed limit
        float max_linear_velocity_;

        float min_linear_velocity_;

        float max_angular_acceleration_;

        float min_angular_acceleration_;

        float current_vehicle_speed_;

        float max_linear_acceleration_;

        float min_linear_acceleration_;

        // vector of linear velocity boundary, first is min,second is max.
        std::vector<std::pair<float, float>> linear_velocity_boundary_;

        std::shared_ptr<PSO> pso_ptr_;
    };
};