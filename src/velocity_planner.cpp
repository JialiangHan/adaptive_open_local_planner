#include "velocity_planner.h"

namespace adaptive_open_local_planner
{

    VelocityPlanner::VelocityPlanner(const float &path_divide_factor, const float &max_linear_velocity, const float &min_linear_velocity, const float &max_angular_acceleration, const float &min_angular_acceleration, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle)
    {
        path_divide_factor_ = path_divide_factor;
        max_linear_velocity_ = max_linear_velocity;
        min_linear_velocity_ = min_linear_velocity;
        max_angular_acceleration_ = max_angular_acceleration;
        min_angular_acceleration_ = min_angular_acceleration;
        weighting_ = weighting;
        personal_learning_rate_ = personal_learning_rate;
        global_learning_rate_ = global_learning_rate;
        cost_difference_boundary_ = cost_difference_boundary;
        max_interation_ = max_interation;
        number_of_particle_ = number_of_particle;
    }

    std::vector<float> VelocityPlanner::planVelocity(const std::vector<Waypoint> &local_path)
    {
        std::vector<std::vector<Waypoint>> divided_path;
        dividePath(local_path, divided_path);
        findVelocityBoundary(divided_path);
        pso_ptr_.reset(new PSO(divided_path, linear_velocity_boundary_, weighting_, personal_learning_rate_, global_learning_rate_, cost_difference_boundary_, max_interation_, number_of_particle_));
        return pso_ptr_->evaluate();
    }

    void VelocityPlanner::dividePath(const std::vector<Waypoint> &local_path, std::vector<std::vector<Waypoint>> &divided_path)
    {
        DLOG(INFO) << "in dividePath.";
        //   find delta curvature of total local path
        float delta_curvature_global = findDeltaCurvature(local_path);
        for (uint i = 0; i < local_path.size(); i++)
        {
            DLOG(INFO) << "index is " << i;
            for (uint j = i + 1; j < local_path.size(); j++)
            {
                float current_delta_curvature = findDeltaCurvature(local_path, i, j);
                if (current_delta_curvature > path_divide_factor_ * delta_curvature_global)
                {
                    if (j != i)
                    {
                        divided_path.emplace_back(PlannerHelpers::extractVector(local_path, i, j));
                        i = j + 1;
                        DLOG(INFO) << "divided path.";
                        break;
                    }
                    else
                    {
                        DLOG(WARNING) << "i equal to j!!!, Something Wrong!!";
                    }
                }

                // if path_divide_factor is one, means no divided path
                if (j == (local_path.size() - 1))
                {
                    DLOG(INFO) << "no path divided";
                    divided_path.emplace_back(local_path);
                    // set i to the end
                    i = local_path.size();
                }
            }
        }
        DLOG(INFO) << "divided_path size is " << divided_path.size();
        DLOG_IF(FATAL, divided_path.size() == 0) << "something wrong, divided path size is zero!!!";
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
                DLOG(WARNING) << "index exceed vector size!!!";
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
                DLOG(WARNING) << "index exceed vector size!!!";
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

    void VelocityPlanner::findVelocityBoundary(const std::vector<std::vector<Waypoint>> &divided_path)
    {
        linear_velocity_boundary_.clear();
        float upper_limit = 0, current_curvature;
        std::pair<float, float> limit_pair;
        for (size_t i = 0; i < divided_path.size(); i++)
        {
            // calculate speed limit for first point of every divided path, and for last one, calculate first and last point
            //  calculate speed limit
            current_curvature = PlannerHelpers::CalculateCurvature(divided_path[i][0], divided_path[i][1]);
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
            // check if this is the last divided path
            if (i == (divided_path.size() - 1))
            {
                int path_length = divided_path[i].size();
                current_curvature = PlannerHelpers::CalculateCurvature(divided_path[i][path_length - 2], divided_path[i][path_length - 1]);
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
        DLOG(INFO) << "linear_velocity_boundary_ size is " << linear_velocity_boundary_.size();
    }
}