/**
 * @file path_evaluator.cpp
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.3
 * @date 2021-12-17
 *
 * @copyright Copyright (c) 2021
 *
 **/
#include "path_evaluator.h"

namespace adaptive_open_local_planner
{
    void PathEvaluator::CallbackPath(const nav_msgs::Path::ConstPtr &path, const std::string &topic_name)
    {
        path_ = PlannerHelpers::convert(path);
        // DLOG(INFO) << "set path";
    }

    void PathEvaluator::CallbackCmd(const geometry_msgs::Twist::ConstPtr &cmd, const std::string &topic_name)
    {
        linear_velocity_vec_.emplace_back(cmd->linear.x);
        angular_velocity_vec_.emplace_back(cmd->angular.z);
        // DLOG(INFO) << "set speed";
    }

    int PathEvaluator::CalculateCurvature()
    {
        if (path_.size() < 3)
        {
            // DLOG(WARNING) << "In CalculateCurvature: path_ does not have enough points!!!";
            return 0;
        }

        curvature_vec_.clear();
        float curvature;
        // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " path_ size is :" << path_.size();

        // use three points to calculate curvature;
        for (uint i = 0; i < path_.size() - 2; ++i)
        {
            // get three points from path_
            Eigen::Vector2f xp(path_[i].x(), path_[i].y());
            // DLOG(INFO) << "xp x is :" << xp(0,0) << "y is: " << xp.y();
            Eigen::Vector2f xi(path_[i + 1].x(), path_[i + 1].y());
            // DLOG(INFO) << "xi x is :" << xi(0,0) << "y is: " << xi.y();
            Eigen::Vector2f xs(path_[i + 2].x(), path_[i + 2].y());

            curvature = PlannerHelpers::CalculateCurvature(xp, xi, xs);
            curvature_vec_.emplace_back(curvature);
            // DLOG(INFO) << "In CalculateCurvature:" << i << "th curvature is:" << curvature;
        }

        return 1;
    }

    int PathEvaluator::CalculateSmoothness()
    {
        if (path_.size() < 3)
        {
            // DLOG(WARNING) << "In CalculateSmoothness: path_ does not have enough points!!!";
            return 0;
        }
        // smoothness = (deltax(i+1)-delta(xi))^2
        // deltax(i+1)= x(i+1)-x(i), the same for deltaxi
        float smoothness;
        for (uint i = 0; i < path_.size() - 2; ++i)
        {
            // get three points from path_
            Eigen::Vector2f xp(path_[i].x(), path_[i].y());
            Eigen::Vector2f xi(path_[i + 1].x(), path_[i + 1].y());
            Eigen::Vector2f xs(path_[i + 2].x(), path_[i + 2].y());
            if (xp == xi || xi == xs)
            {
                DLOG(WARNING) << "In CalculateSmoothness: some points are equal, skip these points for curvature calculation!!";
                continue;
            }
            // get two vector between these three nodes
            Eigen::Vector2f pre_vector = xi - xp;
            Eigen::Vector2f succ_vector = xs - xi;

            smoothness = (succ_vector - pre_vector).norm() * (succ_vector - pre_vector).norm();
            smoothness_vec_.emplace_back(smoothness);
        }

        return 1;
    }

    void PathEvaluator::Plot()
    {
        matplotlibcpp::ion();
        matplotlibcpp::clf();
        std::vector<std::string> title_vec = {"curvature", "smoothness", "Angular velocity", "linear velocity"};
        for (size_t i = 0; i < title_vec.size(); i++)
        {
            matplotlibcpp::subplot(2, 2, i + 1);
            std::vector<float> vec;
            if (title_vec[i] == "curvature")
            {
                vec = curvature_vec_;
            }
            if (title_vec[i] == "smoothness")
            {
                vec = smoothness_vec_;
            }
            if (title_vec[i] == "Angular velocity")
            {
                vec = angular_velocity_vec_;
            }
            if (title_vec[i] == "linear velocity")
            {
                vec = linear_velocity_vec_;
            }

            matplotlibcpp::plot(vec, {{"label", "raw path"}});

            matplotlibcpp::legend({{"loc", "upper right"}});
            // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;

            matplotlibcpp::title(title_vec[i]);
            matplotlibcpp::ylabel(title_vec[i]);
            // matplotlibcpp::ylim(0, 1);
            matplotlibcpp::grid(true);
        }

        matplotlibcpp::pause(0.1);
    }

    void PathEvaluator::EvaluatePath()
    {
        DLOG(INFO) << "in EvaluatePath:";
        CalculateCurvature();
        CalculateSmoothness();
        Plot();
    }
}
