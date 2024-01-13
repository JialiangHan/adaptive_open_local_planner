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

    void PathEvaluator::CallbackOdom(const nav_msgs::Odometry::ConstPtr &odom_msg, const std::string &topic_name)
    {
        // DLOG(INFO) << "x speed is " << odom_msg->twist.twist.linear.x << " y speed is " << odom_msg->twist.twist.linear.y << " z speed is " << odom_msg->twist.twist.linear.z;
        float vehicle_speed_sum =
            std::sqrt(odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x + odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y);

        // DLOG(INFO) << "last speed is " << last_speed_ << " current speed is " << vehicle_speed_sum;
        if (std::abs(vehicle_speed_sum - last_speed_) > 5e-2)
        {
            // DLOG(INFO) << "last speed is " << last_speed_ << " current speed is " << vehicle_speed_sum << " delta is " << std::abs(vehicle_speed_sum - last_speed_);
            actual_velocity_vec_.emplace_back(vehicle_speed_sum);
        }

        // actual_velocity_vec_.emplace_back(actual_speed);
        last_speed_ = vehicle_speed_sum;
        // }
        // move_flag__mutex_.unlock();
    }
    void PathEvaluator::CallbackCmd(const geometry_msgs::Twist::ConstPtr &cmd, const std::string &topic_name)
    {
        // DLOG(INFO) << "x speed is " << cmd->linear.x << " y speed is " << cmd->linear.y << " z speed is " << cmd->linear.z;
        linear_velocity_vec_.emplace_back(cmd->linear.x);
        angular_velocity_vec_.emplace_back(cmd->angular.z);
        // DLOG(INFO) << "set speed";
        float actual_speed = 0;
        // DLOG(INFO) << "move flag is " << move_flag_;
        // move_flag__mutex_.lock();
        // DLOG(INFO) << "move flag is " << move_flag_;
        // if (move_flag_)
        // {
        // DLOG(INFO) << "moving, set speed;";
        actual_speed = std::sqrt(cmd->linear.x * cmd->linear.x + cmd->linear.y * cmd->linear.y);
        // DLOG(INFO) << "last speed is " << last_speed_ << " current speed is " << actual_speed;
        if (std::abs(actual_speed - last_speed_) > 1e-2)
        {
            actual_velocity_vec_.emplace_back(actual_speed);
        }

        // actual_velocity_vec_.emplace_back(actual_speed);
        last_speed_ = actual_speed;
        // }
        // move_flag__mutex_.unlock();
    }

    void PathEvaluator::CallbackPose(const geometry_msgs::PoseStamped::ConstPtr &pose, const std::string &topic_name)
    {
        float current_heading = tf::getYaw(pose->pose.orientation);
        // move_flag__mutex_.lock();
        if (actual_position_x_vec_.size() == 0)
        {
            last_x_ = pose->pose.position.x;
            last_y_ = pose->pose.position.y;
            actual_position_x_vec_.emplace_back(last_x_);
            actual_position_y_vec_.emplace_back(last_y_);
            actual_heading_vec_.emplace_back(current_heading);
            // move_flag_ = true;
            // DLOG(INFO) << "moving.";
        }
        else
        {
            actual_position_x_vec_.emplace_back(pose->pose.position.x);
            actual_position_y_vec_.emplace_back(pose->pose.position.y);
            actual_heading_vec_.emplace_back(current_heading);
            last_x_ = pose->pose.position.x;
            last_y_ = pose->pose.position.y;
            // if (std::abs(last_x_ - pose->pose.position.x) < 1e-1 ||
            //     std::abs(last_y_ - pose->pose.position.y) < 1e-1)
            // {
            //     // move_flag_ = false;
            //     // DLOG(INFO) << "not move.";
            // }
            // else
            // {
            //     // DLOG(INFO) << "moving.";
            //     // move_flag_ = true;
            //     actual_position_x_vec_.emplace_back(pose->pose.position.x);
            //     actual_position_y_vec_.emplace_back(pose->pose.position.y);
            //     actual_heading_vec_.emplace_back(current_heading);
            //     last_x_ = pose->pose.position.x;
            //     last_y_ = pose->pose.position.y;
            // }
        }

        // move_flag__mutex_.unlock();
    }

    void PathEvaluator::CallbackPositionError(const std_msgs::Float32::ConstPtr &position_error, const std::string &topic_name)
    {
        position_error_vec_.emplace_back(position_error->data);
        // DLOG(INFO) << "position_error_vec_ is " << position_error->data;
    }

    void PathEvaluator::CallbackHeadingError(const std_msgs::Float32::ConstPtr &heading_error, const std::string &topic_name)
    {
        heading_error_vec_.emplace_back(heading_error->data);
        // DLOG(INFO) << "heading_error  is " << heading_error->data;
    }

    void PathEvaluator::CallbackVelocityError(const std_msgs::Float32::ConstPtr &velocity_error, const std::string &topic_name)
    {
        velocity_error_vec_.emplace_back(velocity_error->data);
        // DLOG(INFO) << "velocity_error  is " << velocity_error->data;
    }

    void PathEvaluator::CallbackAckermann(const ackermann_msgs::AckermannDriveStamped::ConstPtr &ackermann, const std::string &topic_name)
    {
        command_steering_angle_vec_.emplace_back(ackermann->drive.steering_angle);
        command_speed_vec_.emplace_back(ackermann->drive.speed);
    }

    void PathEvaluator::CallbackJerk(const std_msgs::Float32::ConstPtr &jerk, const std::string &topic_name)
    {
        jerk_vec_.emplace_back(jerk->data);
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

    void PathEvaluator::CalculateCost(const std_msgs::Float32::ConstPtr &cost, const std::string &topic_name)
    {
        cost_vec_.emplace_back(cost->data);
    }

    void PathEvaluator::Plot(const std::vector<std::string> &title_vec)
    {
        // for (const auto &element : title_vec)
        // {
        //     DLOG(INFO) << "element in title vec is " << element;
        // }

        matplotlibcpp::ion();
        matplotlibcpp::clf();
        // std::vector<std::string> title_vec = {"curvature", "smoothness", "Angular velocity", "linear velocity", "jerk", "cost"};
        for (size_t i = 0; i < title_vec.size(); i++)
        {
            matplotlibcpp::subplot(2, 3, i + 1);
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
            if (title_vec[i] == "jerk")
            {
                vec = jerk_vec_;
            }
            if (title_vec[i] == "cost")
            {
                vec = cost_vec_;
            }
            if (title_vec[i] == "position error")
            {
                vec = position_error_vec_;
            }
            if (title_vec[i] == "heading error")
            {
                vec = heading_error_vec_;
            }
            if (title_vec[i] == "velocity error")
            {
                vec = velocity_error_vec_;
            }
            if (title_vec[i] == "steering angle")
            {
                vec = command_steering_angle_vec_;
            }
            if (title_vec[i] == "speed")
            {
                vec = command_speed_vec_;
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
        std::string path = "/home/jialiang/Code/thesis_ws/src/adaptive_open_local_planner/figs/";
        std::string file_name = "mpc";

        auto now = std::time(0);
        std::string time_mark = std::to_string(now);
        std::filesystem::remove_all(path);

        std::filesystem::create_directory(path);
        matplotlibcpp::save(path + file_name + time_mark + ".png");
    }

    void PathEvaluator::EvaluatePath()
    {
        // DLOG(INFO) << "in EvaluatePath:";
        CalculateCurvature();
        CalculateSmoothness();
        std::vector<std::string> title_vec = {"curvature", "smoothness", "Angular velocity", "linear velocity", "jerk", "cost"};
        Plot(title_vec);
    }

    void PathEvaluator::EvaluateControl()
    {
        std::vector<std::string> title_vec = {"position error", "heading error", "velocity error", "speed", "steering angle"};
        Plot(title_vec);
    }

    void PathEvaluator::Plot(const std::vector<Eigen::Vector4d> &path1, const Eigen::MatrixXd &path2)
    {
        // DLOG(INFO) << "Plot";
        matplotlibcpp::ion();
        matplotlibcpp::clf();
        std::vector<float> x1, y1, x2, y2, ref_heading, mpc_heading, ref_velocity, mpc_velocity;
        for (int i = 0; i < path2.cols(); i++)
        {
            x1.emplace_back(path1[i](1));
            y1.emplace_back(path1[i](0));
            ref_heading.emplace_back(path1[i](2));
            ref_velocity.emplace_back(path1[i](3));
            x2.emplace_back(path2.col(i)(1));
            y2.emplace_back(path2.col(i)(0));
            mpc_heading.emplace_back(path2.col(i)(2));
            mpc_velocity.emplace_back(path2.col(i)(3));
            // DLOG(INFO) << "ref path is " << x1[i] << " " << y1[i] << " " << path1[i](2) << " " << path1[i](3);
            // DLOG(INFO) << "mpc path is " << x2[i] << " " << y2[i] << " " << path2.col(i)(2) << " " << path2.col(i)(3);
        }

        matplotlibcpp::subplot(2, 2, 1);
        // path
        matplotlibcpp::plot(x1, y1, {{"label", "ref path"}});
        matplotlibcpp::plot(x2, y2, {{"label", "mpc path"}});
        matplotlibcpp::plot(actual_position_y_vec_, actual_position_x_vec_, {{"label", "actual path"}});
        matplotlibcpp::legend({{"loc", "upper right"}});
        // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;
        matplotlibcpp::ylim(0, 10);
        matplotlibcpp::title("path");
        matplotlibcpp::grid(true);
        matplotlibcpp::subplot(2, 2, 2);
        // heading
        matplotlibcpp::plot(ref_heading, {{"label", "ref heading"}});
        matplotlibcpp::plot(mpc_heading, {{"label", "mpc heading"}});
        matplotlibcpp::plot(actual_heading_vec_, {{"label", "actual heading"}});
        matplotlibcpp::legend({{"loc", "upper right"}});
        // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;
        matplotlibcpp::ylim(-3, 3);
        matplotlibcpp::title("heading");
        matplotlibcpp::grid(true);
        matplotlibcpp::subplot(2, 2, 3);
        // velocity
        matplotlibcpp::plot(ref_velocity, {{"label", "ref velocity"}});
        matplotlibcpp::plot(command_speed_vec_, {{"label", "command velocity"}});
        matplotlibcpp::plot(actual_velocity_vec_, {{"label", "actual velocity"}});
        matplotlibcpp::legend({{"loc", "upper right"}});
        // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;
        matplotlibcpp::ylim(0.0, 1.5);
        matplotlibcpp::title("velocity");
        matplotlibcpp::grid(true);

        matplotlibcpp::pause(0.1);
        std::string file_name = "path";

        std::string path = "/home/jialiang/Code/thesis_ws/src/adaptive_open_local_planner/tests/";

        auto now = std::time(0);
        std::string time_mark = std::to_string(now);

        std::filesystem::create_directory(path);
        matplotlibcpp::save(path + file_name + time_mark + ".png");
    }
}
