/**
 * @file path_evaluator.h
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.1
 * @date 2021-12-07
 *
 * @copyright Copyright (c) 2021
 *
 **/
#pragma once
#include <vector>
#include <string>
#include "ros/ros.h"
#include <unordered_map>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "matplotlibcpp.h"
#include <algorithm>
#include "planner_helpers.h"
#include <std_msgs/Float32.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <filesystem>
#include <boost/thread.hpp>

namespace adaptive_open_local_planner
{
    class PathEvaluator
    {
    public:
        PathEvaluator(){};
        PathEvaluator(const std::string &path_topic, const std::string &cmd_topic, const std::string &jerk_topic, const std::string &cost_topic, const std::string &position_error_topic, const std::string &heading_error_topic, const std::string &velocity_error_topic, const std::string &ackermann_topic, const std::string &pose_topic, const std::string &odom_topic)
        {
            sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic, 1, boost::bind(&PathEvaluator::CallbackPath, this, _1, path_topic));
            // sub_cmd_ = nh_.subscribe<geometry_msgs::Twist>(cmd_topic, 1, boost::bind(&PathEvaluator::CallbackCmd, this, _1, cmd_topic));

            sub_jerk_ = nh_.subscribe<std_msgs::Float32>(jerk_topic, 1, boost::bind(&PathEvaluator::CallbackJerk, this, _1, jerk_topic));
            sub_cost_ = nh_.subscribe<std_msgs::Float32>(cost_topic, 1, boost::bind(&PathEvaluator::CalculateCost, this, _1, cost_topic));

            sub_position_error_ = nh_.subscribe<std_msgs::Float32>(position_error_topic, 100, boost::bind(&PathEvaluator::CallbackPositionError, this, _1, position_error_topic));
            sub_heading_error_ = nh_.subscribe<std_msgs::Float32>(heading_error_topic, 100, boost::bind(&PathEvaluator::CallbackHeadingError, this, _1, heading_error_topic));
            sub_velocity_error_ = nh_.subscribe<std_msgs::Float32>(velocity_error_topic, 100, boost::bind(&PathEvaluator::CallbackVelocityError, this, _1, velocity_error_topic));

            sub_ackermann_ = nh_.subscribe<ackermann_msgs::AckermannDriveStamped>(ackermann_topic, 1, boost::bind(&PathEvaluator::CallbackAckermann, this, _1, ackermann_topic));

            sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>(pose_topic, 1, boost::bind(&PathEvaluator::CallbackPose, this, _1, pose_topic));

            sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&PathEvaluator::CallbackOdom, this, _1, odom_topic));
        };
        void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr &pose, const std::string &topic_name);

        void CallbackOdom(const nav_msgs::Odometry::ConstPtr &odom, const std::string &topic_name);

        void CallbackPath(const nav_msgs::Path::ConstPtr &path, const std::string &topic_name);

        void CallbackCmd(const geometry_msgs::Twist::ConstPtr &cmd, const std::string &topic_name);

        void CallbackJerk(const std_msgs::Float32::ConstPtr &jerk, const std::string &topic_name);

        void CalculateCost(const std_msgs::Float32::ConstPtr &cost, const std::string &topic_name);

        void CallbackPositionError(const std_msgs::Float32::ConstPtr &position_error, const std::string &topic_name);
        void CallbackHeadingError(const std_msgs::Float32::ConstPtr &heading_error, const std::string &topic_name);
        void CallbackVelocityError(const std_msgs::Float32::ConstPtr &velocity_error, const std::string &topic_name);
        void CallbackAckermann(const ackermann_msgs::AckermannDriveStamped::ConstPtr &ackermann, const std::string &topic_name);

        void EvaluatePath();
        /**
         * @brief calculate curvature for the path
         *
         * @param path got from planner
         * @return std::vector<float>
         */
        int CalculateCurvature();

        int CalculateSmoothness();

        void Plot(const std::vector<std::string> &title_vec);

        void EvaluateControl();

        void Plot(const std::vector<Eigen::Vector4d> &path1, const Eigen::MatrixXd &path2);

        void Plot(const std::vector<Eigen::Vector4d> &path1, const std::vector<Eigen::Vector4d> &path2, const std::vector<float> &position_error_vec, const std::vector<float> &heading_error_vec, const std::vector<float> &velocity_error_vec);

    private:
        ros::NodeHandle nh_;

        ros::Subscriber sub_path_;

        ros::Subscriber sub_cmd_;

        ros::Subscriber sub_jerk_;

        ros::Subscriber sub_cost_;

        ros::Subscriber sub_position_error_;
        ros::Subscriber sub_heading_error_;
        ros::Subscriber sub_velocity_error_;
        ros::Subscriber sub_ackermann_;
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_odom_;

        std::vector<Eigen::Vector3f> path_;

        std::vector<float> curvature_vec_;

        std::vector<float> smoothness_vec_;

        std::vector<float> angular_velocity_vec_;

        std::vector<float> linear_velocity_vec_;

        std::vector<float> actual_velocity_vec_;
        std::vector<float> actual_steering_angle_vec_;
        std::vector<float> actual_heading_vec_;
        std::vector<float> actual_position_x_vec_;
        std::vector<float> actual_position_y_vec_;

        float last_heading_ = 100;
        std::vector<float> jerk_vec_;

        std::vector<float> cost_vec_;

        // std::vector<float> path_vec_;

        std::vector<float> position_error_vec_;
        std::vector<float> heading_error_vec_;
        std::vector<float> velocity_error_vec_;
        std::vector<float> command_steering_angle_vec_;
        std::vector<float> command_speed_vec_;

        std::string path_topic_;

        float last_x_, last_y_, last_speed_;

        // bool move_flag_;

        // boost::mutex move_flag__mutex_;
    };
}
