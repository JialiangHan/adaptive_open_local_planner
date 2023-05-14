#include "velocity_planner.h"

namespace adaptive_open_local_planner
{

    VelocityPlanner(const float &max_linear_velocity, const float &min_linear_velocity, const float &max_angular_acceleration, const float &min_angular_acceleration, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate)
    {
        max_linear_velocity_ = max_linear_velocity;
        min_linear_velocity_ = min_linear_velocity;
        max_angular_acceleration_ = max_angular_acceleration;
        min_angular_acceleration_ = min_angular_acceleration;
        weighting_ = weighting;
        personal_learning_rate_ = personal_learning_rate;
        global_learning_rate_ = global_learning_rate;
    }

    std::vector<float> VelocityPlanner::planVelocity(const std::vector<Waypoint> &local_path)
    {
        std::vector<std::vector<Waypoint>> divided_path;
        dividePath(local_path, divided_path);
        findVelocityBoundary(local_path);
        pso_ptr_.reset(new PSO(divided_path, linear_velocity_boundary_, weighting_, personal_learning_rate_, global_learning_rate_));
        return pso_ptr_->evaluate();
    }

    void VelocityPlanner::dividePath(const std::vector<Waypoint> &local_path, std::vector<std::vector<Waypoint>> &divided_path)
    {
        // find delta curvature of total local path
        float delta_curvature_global = findDeltaCurvature(local_path);
        // parameter which control how to divide path //TODO set K to a parameter
        float K = 0;
        for (uint i = 0; i < local_path.size(); i++)
        {
            for (uint j = i + 1; j < local_path.size(); j++)
            {
                float current_delta_curvature = findDeltaCurvature(local_path, i, j);
                if (current_delta_curvature > K * delta_curvature_global)
                {
                    if ((j - 1) != i)
                    {
                        divided_path.emplace_back(PlannerHelpers::extractVector(local_path, i, j - 1));
                        i = j;
                        break;
                    }
                    else
                    {
                        DLOG(WARNING) << "i equal to j-1!!!, Something Wrong!!";
                    }
                }
            }
        }
    }

    float VelocityPlanner::findDeltaCurvature(const std::vector<Waypoint> &local_path)
    {
        float max_curvature, min_curvature;
        findMaxCurvature(local_path, max_curvature);
        findMinCurvature(local_path, min_curvature);
        return max_curvature - min_curvature;
    }

    float VelocityPlanner::findDeltaCurvature(const std::vector<Waypoint> &local_path, int start_index, int end_index)
    {
        float max_curvature, min_curvature;
        findMaxCurvature(local_path, max_curvature, start_index, end_index);
        findMinCurvature(local_path, min_curvature, start_index, end_index);
        return max_curvature - min_curvature;
    }

    void VelocityPlanner::findMaxCurvature(const std::vector<Waypoint> &local_path, float &max_curvature)
    {
        findMaxCurvature(local_path, max_curvature, 0, local_path.size() - 1);
    }

    void VelocityPlanner::findMaxCurvature(const std::vector<Waypoint> &local_path, float &max_curvature, int start_index, int end_index)
    {
        max_curvature = 0;
        float current_curvature;
        for (uint i = start_index; i < end_index; i++)
        {
            if ((i + 1) >= local_path.size())
            {
                DLOG(WARING) << "index exceed vector size!!!";
            }
            else
            {
                current_curvature = PlannerHelpers::CalculateCurvature(local_path[i], local_path[i + 1]);
                if (current_curvature > max_curvature)
                {
                    max_curvature = current_curvature;
                }
            }
        }
    }

    void VelocityPlanner::findMinCurvature(const std::vector<Waypoint> &local_path, float &min_curvature)
    {
        findMinCurvature(local_path, min_curvature, 0, local_path.size() - 1);
    }

    void VelocityPlanner::findMinCurvature(const std::vector<Waypoint> &local_path, float &min_curvature, int start_index, int end_index)
    {
        min_curvature = 0;
        float current_curvature;
        for (uint i = start_index; i < end_index; i++)
        {
            if ((i + 1) >= local_path.size())
            {
                DLOG(WARING) << "index exceed vector size!!!";
            }
            else
            {
                current_curvature = PlannerHelpers::CalculateCurvature(local_path[i], local_path[i + 1]);
                if (current_curvature < min_curvature)
                {
                    min_curvature = current_curvature;
                }
            }
        }
    }

    void VelocityPlanner::findVelocityBoundary(const std::vector<Waypoint> &local_path)
    {
        linear_velocity_boundary_.clear();
        float upper_limit = 0, current_curvature;
        std::pair<float, float> limit_pair;
        for (size_t i = 0; i < local_path.size() - 1; i++)
        {
            // calculate speed limit
            current_curvature = PlannerHelpers::CalculateCurvature(local_path[i], local_path[i + 1]);
            if (current_curvature != 0)
            {
                upper_limit = std::sqrt(max_angular_acceleration_ / current_curvature);
            }
            else
            {
                DLOG(WARNING) << "current_curvature is zero!!!";
                // set to a large value
                upper_limit = 100000;
            }
            // compare with pre set limit
            if (upper_limit < max_linear_velocity_)
            {
                limit_pair.second = upper_limit;
            }
            else
            {
                limit_pair.second = max_linear_velocity_;
            }
            limit_pair.first = min_linear_velocity_;

            linear_velocity_boundary_.emplace_back(limit_pair);
        }
    }
}