#include "adaptive_open_local_planner_ros.h"
#include <pluginlib/class_list_macros.h>

// register this planner both as a BaseLocalPlanner
PLUGINLIB_EXPORT_CLASS(adaptive_open_local_planner::AdaptiveOpenLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace adaptive_open_local_planner
{

    AdaptiveOpenLocalPlannerROS::AdaptiveOpenLocalPlannerROS()
    {
        std::string log_dir = "/home/jialiang/Code/thesis_ws/src/adaptive_open_local_planner/log/adaptive_open_local_planner_";
        for (int severity = 0; severity < google::NUM_SEVERITIES; ++severity)
        {
            google::SetLogDestination(severity, log_dir.c_str());
            google::SetLogSymlink(severity, log_dir.c_str());
        }
        google::InitGoogleLogging("adaptive_open_local_planner");

        google::InstallFailureSignalHandler();

        google::EnableLogCleaner(5);
        FLAGS_alsologtostderr = 1;
        // DLOG(INFO) << "creating adaptive_open_local_planner planner";
    }
    AdaptiveOpenLocalPlannerROS::AdaptiveOpenLocalPlannerROS(std::string name, tf2_ros::Buffer *tf,
                                                             costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        std::string log_dir = "/home/jialiang/Code/thesis_ws/src/adaptive_open_local_planner/log/adaptive_open_local_planner_";
        for (int severity = 0; severity < google::NUM_SEVERITIES; ++severity)
        {
            google::SetLogDestination(severity, log_dir.c_str());
            google::SetLogSymlink(severity, log_dir.c_str());
        }
        google::InitGoogleLogging("adaptive_open_local_planner");

        google::InstallFailureSignalHandler();

        google::EnableLogCleaner(5);
        FLAGS_alsologtostderr = 1;
        // DLOG(INFO) << "creating adaptive_open_local_planner planner";
        initialize(name, tf, costmap_ros);
    }
    void AdaptiveOpenLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            // create Node Handle with name of plugin (as used in move_base for loading)
            ros::NodeHandle nh_para("~/" + name);
            ros::NodeHandle nh;
            params_.loadRosParamFromNodeHandle(nh_para);
            // DLOG(INFO) << "evaluate_path is " << params_.evaluate_path;
            if (params_.evaluate_path)
            {
                std::string path_topic, cmd_topic, jerk_topic, cost_topic, position_error_topic, heading_error_topic, velocity_error_topic;
                path_topic = "extracted_path_rviz";
                cmd_topic = "cmd_vel";
                jerk_topic = "jerk";
                cost_topic = "cost";
                position_error_topic = "position_error";
                heading_error_topic = "heading_error";
                velocity_error_topic = "velocity_error";

                path_evaluator_ptr_.reset(new PathEvaluator(path_topic, cmd_topic, jerk_topic, cost_topic, position_error_topic, heading_error_topic, velocity_error_topic, params_.ackermann_cmd_topic, params_.current_pose_rviz_topic, params_.odom_topic));
            }
            mpc_.initialize(params_.wheelbase_length, 1 / params_.planning_frequency, params_.control_delay, params_.predicted_length, params_.heading_weighting, params_.last_heading_weighting, params_.speed_weighting, params_.max_linear_velocity, params_.max_linear_acceleration, params_.max_steer_angle, params_.max_angular_acceleration, params_.evaluate_path);
            // DLOG(INFO) << "max linear velocity is " << params_.max_linear_velocity;

            velocity_planner_ptr_.reset(new VelocityPlanner(params_.path_divide_factor, params_.max_linear_velocity, params_.min_linear_velocity, params_.max_angular_acceleration, params_.min_angular_acceleration, params_.max_linear_acceleration, params_.min_linear_acceleration, params_.weighting, params_.personal_learning_rate, params_.global_learning_rate, params_.cost_difference_boundary, params_.max_interation, params_.number_of_particle));

            // Subscribe & Advertise
            odom_sub = nh.subscribe(params_.odom_topic, 1, &AdaptiveOpenLocalPlannerROS::odomCallback, this);

            extracted_path_rviz_pub = nh.advertise<nav_msgs::Path>(params_.extracted_path_rviz_topic, 1, true);

            global_plan_rviz_pub = nh.advertise<nav_msgs::Path>("global_plan", 1, true);

            current_pose_rviz_pub = nh.advertise<geometry_msgs::PoseStamped>(params_.current_pose_rviz_topic, 1, true);
            roll_outs_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(params_.roll_outs_rviz_topic, 1, true);
            weighted_trajectories_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(params_.weighted_trajectories_rviz_topic, 1, true);
            safety_box_rviz_pub = nh.advertise<visualization_msgs::Marker>(params_.safety_box_rviz_topic, 1, true);
            car_footprint_rviz_pub = nh.advertise<visualization_msgs::Marker>(params_.car_footprint_rviz_topic, 1, true);
            box_obstacle_rviz_pub = nh.advertise<visualization_msgs::Marker>(params_.box_obstacle_rviz_topic, 1, true);

            ackermann_cmd_mux_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>(params_.ackermann_cmd_topic, 100, true);

            global_plan_received_ = false;
            vehicle_state_received_ = false;

            prev_cost_ = 0;

            // reserve some memory for obstacles
            obstacles_.reserve(500);

            // init other variables
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

            global_frame_ = costmap_ros_->getGlobalFrameID();

            robot_base_frame_ = costmap_ros_->getBaseFrameID();

            // set initialized flag
            initialized_ = true;

            // DLOG(INFO) << "adaptive_open_local_planner plugin initialized.";
        }
    }

    bool AdaptiveOpenLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        // check if plugin is initialized
        if (!initialized_)
        {
            ROS_ERROR("adaptive_open_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        // store the global plan
        global_plan_.clear();

        global_plan_ = orig_global_plan;
        global_plan_received_ = true;

        // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
        // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

        // reset goal_reached_ flag
        goal_reached_ = false;
        nav_msgs::Path global_path;
        PlannerHelpers::convert(orig_global_plan, global_path);
        global_plan_rviz_pub.publish(global_path);

        return true;
    }

    bool AdaptiveOpenLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {

        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped cmd_vel_stamped;
        bool outcome = computeVelocityCommands(dummy_pose, cmd_vel_stamped);
        cmd_vel = cmd_vel_stamped.twist;
        // DLOG(INFO) << "velocity is " << cmd_vel.linear.x << " " << cmd_vel.linear.y << " steering angle rate is " << cmd_vel.angular.z;
        if (params_.evaluate_path)
        {
            // path_evaluator_ptr_->EvaluatePath();
            path_evaluator_ptr_->EvaluateControl();
        }

        return outcome;
    }

    bool AdaptiveOpenLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped &pose, geometry_msgs::TwistStamped &cmd_vel)
    {
        // check if plugin initialized
        if (!initialized_)
        {
            ROS_ERROR("adaptive_open_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        static uint32_t seq = 0;
        cmd_vel.header.seq = seq++;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = robot_base_frame_;
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        goal_reached_ = false;

        // Get robot pose
        geometry_msgs::PoseStamped robot_pose;
        costmap_ros_->getRobotPose(robot_pose);

        convertObstacle();
        if (!global_plan_received_)
        {
            ROS_WARN("Global path not received!");
            return false;
        }
        if (!vehicle_state_received_)
        {
            ROS_WARN("Odom data not received!");
            return false;
        }
        // prune global plan to cut off parts of the past (spatially before the robot)
        pruneGlobalPlan(*tf_, robot_pose, global_plan_, params_.global_plan_prune_distance);

        std::vector<Waypoint> extracted_path, best_path;
        extractGlobalPathSection(extracted_path);

        std::vector<std::vector<Waypoint>> roll_outs;
        generateRollOuts(extracted_path, roll_outs);

        std::vector<PathCost> trajectory_costs;
        doOneStepStatic(roll_outs, extracted_path, trajectory_costs, best_path);
        float velocity, steering_angle_rate;
        std::vector<Waypoint> waypoint_vec = calculateVelocityAndSteeringAngleRate(best_path, velocity, steering_angle_rate);
        std::vector<Eigen::Vector4d> trajectory = convertTrajectory(waypoint_vec);
        Eigen::Vector4d current_state(current_state_in_map_frame_.x, current_state_in_map_frame_.y, current_state_in_map_frame_.yaw, current_state_in_map_frame_.speed);
        trajectory = fakeTrajectory(current_state);
        mpc_.inputRefTrajectory(trajectory);

        Eigen::Vector2d control_vec = mpc_.output(current_state);
        // goalCheck();
        goalCheck(trajectory);

        Eigen::MatrixXd predictedState = mpc_.getPredictedState();

        path_evaluator_ptr_->Plot(trajectory, predictedState);
        // cmd_vel.twist.linear.x = current_state_in_map_frame_.speed + control_vec[0] * 1 / params_.planning_frequency;
        // cmd_vel.twist.linear.y = 0;
        // cmd_vel.twist.angular.z = control_vec[1] * params_.planning_frequency;

        // DLOG(INFO) << "speed command is " << cmd_vel.twist.linear.x << " steering angle command is " << cmd_vel.twist.angular.z;
        publishAckermanncmdstate(predictedState.col(0));
        // publishAckermanncmd(control_vec);

        return true;
    }
    bool AdaptiveOpenLocalPlannerROS::publishAckermanncmdstate(const Eigen::Vector4d &predicted_state)
    {

        double speed = predicted_state[3];
        // control_vec[1] is already steering angle, no need to times planning freq
        // double turn = control_vec[1] * params_.planning_frequency;
        double turn = predicted_state[2] - current_state_in_map_frame_.yaw;
        // DLOG(INFO) << "current speed is " << current_state_in_map_frame_.speed << " predicted speed is " << speed;
        DLOG(INFO) << "speed command is " << speed << " steering angle command is " << turn;

        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";

        msg.drive.speed = speed;
        // no use
        msg.drive.acceleration = 0;
        // no use
        msg.drive.jerk = 1;
        msg.drive.steering_angle = turn;
        // no use
        msg.drive.steering_angle_velocity = 0;

        ackermann_cmd_mux_pub_.publish(msg);
        return true;
    }
    bool AdaptiveOpenLocalPlannerROS::publishAckermanncmd(const Eigen::Vector2d &control_vec)
    {

        double speed = current_state_in_map_frame_.speed + control_vec[0] * 1 / params_.planning_frequency;
        // control_vec[1] is already steering angle, no need to times planning freq
        // double turn = control_vec[1] * params_.planning_frequency;
        double turn = control_vec[1];
        DLOG(INFO) << "current speed is " << current_state_in_map_frame_.speed << " predicted acceleration is " << control_vec[0] << " predicted speed is " << speed;
        DLOG(INFO) << "speed command is " << speed << " steering angle command is " << turn << " acceleration is " << control_vec[0] << " steering angle rate is " << control_vec[1];

        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_link";

        msg.drive.speed = speed;
        msg.drive.acceleration = control_vec[0];
        // no use
        msg.drive.jerk = 1;
        msg.drive.steering_angle = turn;
        // no use
        msg.drive.steering_angle_velocity = 0;

        ackermann_cmd_mux_pub_.publish(msg);
        return true;
    }

    bool AdaptiveOpenLocalPlannerROS::isGoalReached()
    {
        if (goal_reached_)
        {
            ROS_INFO("GOAL Reached!");

            return true;
        }
        return false;
    }

    void AdaptiveOpenLocalPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        vehicle_state_received_ = true;
        // DLOG(INFO) << "odom received.";
        // set vehicle speed to be sqrt(x^2+y^2)
        float vehicle_speed_sum =
            std::sqrt(odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x + odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y);
        // DLOG(INFO) << "current vehicle speed x is " << current_state_in_map_frame_.speed << " y is " << odom_msg->twist.twist.linear.y << " z is " << odom_msg->twist.twist.linear.z << " x^2+y^2= " << vehicle_speed_sum;
        current_state_in_map_frame_.speed = vehicle_speed_sum;
        if (fabs(odom_msg->twist.twist.linear.x) > 0.25)
            current_state_in_map_frame_.steer = atan(params_.wheelbase_length * odom_msg->twist.twist.angular.z / odom_msg->twist.twist.linear.x);

        if (initialized_)
        {
            // Get robot pose
            geometry_msgs::PoseStamped robot_pose;
            costmap_ros_->getRobotPose(robot_pose);
            robot_pose_ = PoseSE2(robot_pose.pose);

            current_state_in_map_frame_.yaw = tf::getYaw(robot_pose.pose.orientation);

            current_state_in_map_frame_.x = robot_pose.pose.position.x;
            current_state_in_map_frame_.y = robot_pose.pose.position.y;

            VisualizationHelpers::createCurrentPoseMarker(current_state_in_map_frame_, robot_pose);
            current_pose_rviz_pub.publish(robot_pose);
        }
    }
    // checked, fine
    void AdaptiveOpenLocalPlannerROS::extractGlobalPathSection(std::vector<Waypoint> &extracted_path)
    {
        // DLOG(INFO) << "In extractGlobalPathSection:";
        if (global_plan_.size() < 2)
            return;

        extracted_path.clear();
        float resolution = costmap_->getResolution();
        Waypoint car_pos;
        car_pos.x = current_state_in_map_frame_.x;
        car_pos.y = current_state_in_map_frame_.y;
        car_pos.heading = current_state_in_map_frame_.yaw;
        int closest_index = PlannerHelpers::getClosestNextWaypointIndex(PlannerHelpers::convert(global_plan_), car_pos);

        if (closest_index + 1 >= global_plan_.size())
            closest_index = global_plan_.size() - 2;

        double d = 0;

        for (int i = closest_index; i < (int)global_plan_.size(); i++)
        {
            Waypoint point = PlannerHelpers::convert(global_plan_[i]);

            extracted_path.push_back(point);
            if (i > 0)
            {
                Waypoint previous_point = PlannerHelpers::convert(global_plan_[i - 1]);
                d += hypot(point.x - previous_point.x, point.y - previous_point.y);
            }

            if (d > params_.max_local_plan_distance * resolution * 6)
                break;
        }

        // for (int i = 0; i < (int)extracted_path.size(); i++)
        // {
        //     DLOG(INFO) << i << "th element in extracted path is " << extracted_path[i].x << " " << extracted_path[i].y << " " << extracted_path[i].heading;
        // }

        if (extracted_path.size() < 2)
        {
            ROS_WARN("Adaptive Open Local Planner Node: Extracted Global Plan is too small, Size = %d", (int)extracted_path.size());
            return;
        }

        PlannerHelpers::fixPathDensity(extracted_path, params_.path_density * resolution * 6);
        // for (const auto &element : extracted_path)
        // {
        //     DLOG(INFO) << "element in extracted path after fixPathDensity is " << element.x << " " << element.y << " " << element.heading;
        // }
        PlannerHelpers::smoothPath(extracted_path, params_.smooth_tolerance, params_.smooth_data_weight, params_.smooth_weight);
        // for (const auto &element : extracted_path)
        // {
        //     DLOG(INFO) << "element in extracted path after smoothPath is " << element.x << " " << element.y << " " << element.heading;
        // }
        PlannerHelpers::calculateAngleAndCost(extracted_path, prev_cost_);
        // for (const auto &element : extracted_path)
        // {
        //     DLOG(INFO) << "element in extracted path after calculateAngleAndCost is " << element.x << " " << element.y << " " << element.heading;
        // }

        calculateLinearVelocity(extracted_path);

        nav_msgs::Path path;
        VisualizationHelpers::createExtractedPathMarker(extracted_path, path);
        extracted_path_rviz_pub.publish(path);
    }

    void AdaptiveOpenLocalPlannerROS::generateRollOuts(const std::vector<Waypoint> &path, std::vector<std::vector<Waypoint>> &roll_outs)
    {
        // DLOG(INFO) << "path size: " << path.size() ;
        if (path.size() == 0)
            return;
        if (params_.max_local_plan_distance <= 0)
            return;
        roll_outs.clear();

        float resolution = costmap_->getResolution();

        int i_limit_index = (params_.sampling_tip_margin / 0.3) / params_.path_density;
        if (i_limit_index >= path.size())
            i_limit_index = path.size() - 1;
        // DLOG(INFO) << "i_limit_index: " << i_limit_index ;

        double initial_roll_in_distance;

        RelativeInfo info;
        Waypoint car_pos;
        car_pos.x = current_state_in_map_frame_.x;
        car_pos.y = current_state_in_map_frame_.y;
        car_pos.heading = current_state_in_map_frame_.yaw;
        PlannerHelpers::getRelativeInfo(path, car_pos, info);
        initial_roll_in_distance = info.perp_distance;
        // DLOG(INFO) << "initial_roll_in_distance: " << initial_roll_in_distance;

        double remaining_distance = 0;
        for (int i = 0; i < path.size() - 1; i++)
        {
            remaining_distance += distance2points(path[i], path[i + 1]);
        }
        // DLOG(INFO)) << "remaining_distance: " << remaining_distance ;

        // calculate the starting index
        double d_limit = 0;
        int start_index = 0, end_index = 0;

        // calculate end index
        double start_distance = params_.roll_in_speed_factor * current_state_in_map_frame_.speed + params_.roll_in_margin * resolution * 6;
        if (start_distance > remaining_distance)
            start_distance = remaining_distance;
        // DLOG(INFO) << "start_distance: " << start_distance;

        d_limit = 0;
        for (int i = 0; i < path.size() - 1; i++)
        {
            d_limit += distance2points(path[i], path[i + 1]);

            if (d_limit >= start_distance)
            {
                // DLOG(INFO) << "d_limit is " << d_limit;
                end_index = i;
                break;
            }
        }
        // DLOG(INFO) << "end_index: " << end_index;

        int central_trajectory_index = params_.roll_outs_number / 2;
        std::vector<double> end_laterals;
        // TODO make this distance adaptive
        //  distance from roll outs to global plan in vertical direction
        double end_roll_in_distance;
        for (int i = 0; i < params_.roll_outs_number + 1; i++)
        {
            end_roll_in_distance = params_.roll_out_density * (i - central_trajectory_index);
            end_laterals.push_back(end_roll_in_distance * resolution * 5);
            // DLOG(INFO) << "roll out num: " << i << ", end_roll_in_distance: " << end_roll_in_distance * resolution * 5;
        }

        // calculate the actual calculation starting index
        d_limit = 0;
        int smoothing_start_index = start_index;
        int smoothing_end_index = end_index;

        for (int i = smoothing_start_index; i < path.size(); i++)
        {
            if (i > 0)
                d_limit += distance2points(path[i], path[i - 1]);
            if (d_limit > params_.sampling_tip_margin * resolution * 6)
                break;

            smoothing_start_index++;
        }

        d_limit = 0;
        for (int i = end_index; i < path.size(); i++)
        {
            if (i > 0)
                d_limit += distance2points(path[i], path[i - 1]);
            if (d_limit > params_.sampling_tip_margin * resolution * 6)
                break;

            smoothing_end_index++;
        }
        // DLOG(INFO) << "start_index: " << start_index << ", end_index: " << end_index << ", smoothing_start_index: " << smoothing_start_index << ", smoothing_end_index: " << smoothing_end_index;

        int nSteps = end_index - smoothing_start_index;
        // DLOG(INFO) << "nSteps: " << nSteps;

        std::vector<double> inc_list;
        std::vector<double> inc_list_inc;
        for (int i = 0; i < params_.roll_outs_number + 1; i++)
        {
            double diff = end_laterals[i] - initial_roll_in_distance;
            // DLOG(INFO) << "diff: " << diff;
            inc_list.push_back(diff / (double)nSteps);
            roll_outs.push_back(std::vector<Waypoint>());
            inc_list_inc.push_back(0);
        }

        std::vector<std::vector<Waypoint>> excluded_from_smoothing;
        for (int i = 0; i < params_.roll_outs_number + 1; i++)
            excluded_from_smoothing.push_back(std::vector<Waypoint>());

        Waypoint wp;
        // Insert first straight points within the tip of the car range
        for (int j = start_index; j < smoothing_start_index; j++)
        {
            wp = path[j];
            double original_speed = wp.speed;
            for (int i = 0; i < params_.roll_outs_number + 1; i++)
            {
                wp.x = path[j].x - initial_roll_in_distance * cos(wp.heading + M_PI_2);
                wp.y = path[j].y - initial_roll_in_distance * sin(wp.heading + M_PI_2);
                if (i != central_trajectory_index)
                    wp.speed = original_speed * params_.lane_change_speed_factor;
                else
                    wp.speed = original_speed;

                if (j < i_limit_index)
                    excluded_from_smoothing[i].push_back(wp);
                else
                    roll_outs[i].push_back(wp);
            }
        }

        for (int j = smoothing_start_index; j < end_index; j++)
        {
            wp = path[j];
            double original_speed = wp.speed;
            for (int i = 0; i < params_.roll_outs_number + 1; i++)
            {
                inc_list_inc[i] += inc_list[i];
                double d = inc_list_inc[i];
                wp.x = path[j].x - initial_roll_in_distance * cos(wp.heading + M_PI_2) - d * cos(wp.heading + M_PI_2);
                wp.y = path[j].y - initial_roll_in_distance * sin(wp.heading + M_PI_2) - d * sin(wp.heading + M_PI_2);

                if (i != central_trajectory_index)
                    wp.speed = original_speed * params_.lane_change_speed_factor;
                else
                    wp.speed = original_speed;

                roll_outs[i].push_back(wp);
            }
        }

        // Insert last straight points to make better smoothing
        for (int j = end_index; j < smoothing_end_index; j++)
        {
            wp = path[j];
            double original_speed = wp.speed;
            for (int i = 0; i < params_.roll_outs_number + 1; i++)
            {
                double d = end_laterals[i];
                wp.x = path[j].x - d * cos(wp.heading + M_PI_2);
                wp.y = path[j].y - d * sin(wp.heading + M_PI_2);
                if (i != central_trajectory_index)
                    wp.speed = original_speed * params_.lane_change_speed_factor;
                else
                    wp.speed = original_speed;
                roll_outs[i].push_back(wp);
            }
        }

        for (int i = 0; i < params_.roll_outs_number + 1; i++)
            roll_outs[i].insert(roll_outs[i].begin(), excluded_from_smoothing[i].begin(), excluded_from_smoothing[i].end());

        d_limit = 0;
        for (int j = smoothing_end_index; j < path.size(); j++)
        {
            if (j > 0)
                d_limit += distance2points(path[j], path[j - 1]);

            if (d_limit > params_.max_local_plan_distance * resolution * 6) // max_roll_distance)
                break;

            wp = path[j];
            double original_speed = wp.speed;
            for (int i = 0; i < roll_outs.size(); i++)
            {
                double d = end_laterals[i];
                wp.x = path[j].x - d * cos(wp.heading + M_PI_2);
                wp.y = path[j].y - d * sin(wp.heading + M_PI_2);

                if (i != central_trajectory_index)
                    wp.speed = original_speed * params_.lane_change_speed_factor;
                else
                    wp.speed = original_speed;

                roll_outs[i].push_back(wp);
            }
        }

        for (int i = 0; i < params_.roll_outs_number + 1; i++)
        {
            PlannerHelpers::smoothPath(roll_outs[i], params_.smooth_tolerance, params_.smooth_data_weight, params_.smooth_weight);
            PlannerHelpers::calculateAngleAndCost(roll_outs[i], prev_cost_);
            PlannerHelpers::predictConstantTimeCostForTrajectory(roll_outs[i], current_state_in_map_frame_);
        }

        visualization_msgs::MarkerArray roll_out_marker_array;
        VisualizationHelpers::createRollOutsMarker(roll_outs, roll_out_marker_array);
        roll_outs_rviz_pub.publish(roll_out_marker_array);
    }

    void AdaptiveOpenLocalPlannerROS::doOneStepStatic(const std::vector<std::vector<Waypoint>> &roll_outs, const std::vector<Waypoint> &extracted_path, std::vector<PathCost> &trajectory_costs, std::vector<Waypoint> &best_path)
    {

        RelativeInfo car_info;
        Waypoint car_pos;
        car_pos.x = current_state_in_map_frame_.x;
        car_pos.y = current_state_in_map_frame_.y;
        car_pos.heading = current_state_in_map_frame_.yaw;
        PlannerHelpers::getRelativeInfo(extracted_path, car_pos, car_info);
        int curr_index = params_.roll_outs_number / 2 + floor(car_info.perp_distance / params_.roll_out_density);
        // DLOG(INFO) <<  "Current Index: " << curr_index ;
        if (curr_index < 0)
            curr_index = 0;
        else if (curr_index > params_.roll_outs_number)
            curr_index = params_.roll_outs_number;

        trajectory_costs.clear();
        if (roll_outs.size() > 0)
        {
            PathCost tc;
            int central_index = params_.roll_outs_number / 2;
            tc.lane_index = 0;
            for (int it = 0; it < roll_outs.size(); it++)
            {
                tc.index = it;
                tc.relative_index = it - central_index;
                tc.distance_from_center = params_.roll_out_density * tc.relative_index;
                tc.priority_cost = fabs(tc.distance_from_center);
                tc.closest_obj_distance = params_.horizon_distance;
                // if(roll_outs[it].size() > 0)
                //     tc.lane_change_cost = roll_outs[it][0].lane_change_cost;
                tc.bBlocked = false;
                trajectory_costs.push_back(tc);
            }
        }

        PlannerHelpers::calculateTransitionCosts(trajectory_costs, curr_index, params_.roll_out_density);

        std::vector<Waypoint> contour_points;
        for (int io = 0; io < box_obstacles.size(); io++)
        {
            Mat3 rotationMat(box_obstacles[io].heading - M_PI_2);
            Mat3 translationMat(box_obstacles[io].x, box_obstacles[io].y);

            Waypoint bottom_left;
            bottom_left.x = -box_obstacles[io].width / 2.0;
            bottom_left.y = -box_obstacles[io].length / 2.0;

            Waypoint bottom_right;
            bottom_right.x = box_obstacles[io].width / 2.0;
            bottom_right.y = -box_obstacles[io].length / 2.0;

            Waypoint top_right;
            top_right.x = box_obstacles[io].width / 2.0;
            top_right.y = box_obstacles[io].length / 2.0;

            Waypoint top_left;
            top_left.x = -box_obstacles[io].width / 2.0;
            top_left.y = box_obstacles[io].length / 2.0;

            bottom_left = rotationMat * bottom_left;
            bottom_left = translationMat * bottom_left;

            bottom_right = rotationMat * bottom_right;
            bottom_right = translationMat * bottom_right;

            top_right = rotationMat * top_right;
            top_right = translationMat * top_right;

            top_left = rotationMat * top_left;
            top_left = translationMat * top_left;

            contour_points.push_back(bottom_left);
            contour_points.push_back(bottom_right);
            contour_points.push_back(top_right);
            contour_points.push_back(top_left);
        }

        visualization_msgs::Marker car_footprint_marker, safety_box_marker;
        PlannerHelpers::calculateLateralAndLongitudinalCostsStatic(trajectory_costs, roll_outs, extracted_path, contour_points,
                                                                   current_state_in_map_frame_, car_footprint_marker, safety_box_marker,
                                                                   params_.vehicle_length, params_.vehicle_width,
                                                                   params_.wheelbase_length, params_.horizontal_safety_distance,
                                                                   params_.vertical_safety_distance, params_.max_steer_angle,
                                                                   params_.min_following_distance, params_.lateral_skip_distance);

        car_footprint_rviz_pub.publish(car_footprint_marker);
        safety_box_rviz_pub.publish(safety_box_marker);

        PlannerHelpers::calculateCurvatureCosts(trajectory_costs, roll_outs);

        PlannerHelpers::normalizeCosts(trajectory_costs, params_.priority_weight, params_.transition_weight, params_.lat_weight, params_.long_weight, params_.curvature_weight);

        int smallestIndex = -1;
        double smallestCost = DBL_MAX;
        double smallestDistance = DBL_MAX;
        double velo_of_next = 0;

        // cout << "Trajectory Costs Log : CurrIndex: " << currIndex << " --------------------- " << endl;
        for (int ic = 0; ic < trajectory_costs.size(); ic++)
        {
            // cout << m_PathCosts.at(ic).ToString();
            if (!trajectory_costs[ic].bBlocked && trajectory_costs[ic].cost < smallestCost)
            {
                smallestCost = trajectory_costs[ic].cost;
                smallestIndex = ic;
            }

            if (trajectory_costs[ic].closest_obj_distance < smallestDistance)
            {
                smallestDistance = trajectory_costs[ic].closest_obj_distance;
                velo_of_next = trajectory_costs[ic].closest_obj_velocity;
            }
        }
        // cout << "Smallest Distance: " <<  smallestDistance << "------------------------------------------------------------- " << endl;

        if (smallestIndex == -1)
        {
        }
        else if (smallestIndex >= 0)
        {
            best_path = roll_outs[smallestIndex];
        }

        visualization_msgs::MarkerArray weighted_roll_out_marker_array;
        VisualizationHelpers::createWeightedRollOutsMarker(roll_outs, trajectory_costs, smallestIndex, weighted_roll_out_marker_array);
        weighted_trajectories_rviz_pub.publish(weighted_roll_out_marker_array);
    }

    void AdaptiveOpenLocalPlannerROS::updateObstacleContainerWithCostmap()
    {
        // Add costmap obstacles if desired
        Eigen::Vector2d robot_orient(std::cos(current_state_in_map_frame_.yaw), std::sin(current_state_in_map_frame_.yaw));
        Eigen::Vector2d robot_pose(current_state_in_map_frame_.x, current_state_in_map_frame_.y);
        for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i)
        {
            for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j)
            {
                if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
                {
                    Eigen::Vector2d obs;
                    costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                    // check if obstacle is interesting (e.g. not far behind the robot)
                    Eigen::Vector2d obs_dir = obs - robot_pose;
                    // if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > params_.obstacles.costmap_obstacles_behind_robot_dist)
                    //     continue;
                    if (obs_dir.dot(robot_orient) < 0)
                        continue;
                    obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
                }
            }
        }
    }

    void AdaptiveOpenLocalPlannerROS::convertObstacle()
    {
        circle_obstacles.clear();
        box_obstacles.clear();
        float resolution = costmap_->getResolution();

        for (size_t i = 0; i < obstacles_.size(); i++)
        {
            CircleObstacle co;
            co.x = obstacles_[i]->getCentroid().x();
            co.y = obstacles_[i]->getCentroid().y();
            co.vx = 0;
            co.vy = 0;
            co.radius = resolution;
            co.true_radius = resolution;
            circle_obstacles.push_back(co);

            BoxObstacle bo;
            bo.x = obstacles_[i]->getCentroid().x();
            bo.y = obstacles_[i]->getCentroid().y();
            bo.heading = 0;
            bo.width = resolution;
            bo.length = resolution;
            box_obstacles.push_back(bo);

            visualization_msgs::Marker box_obstacle_marker;
            VisualizationHelpers::createBoxObstacleMarker(box_obstacles[i], box_obstacle_marker);
            box_obstacle_rviz_pub.publish(box_obstacle_marker);
        }
    }

    std::vector<Waypoint> AdaptiveOpenLocalPlannerROS::calculateVelocityAndSteeringAngleRate(const std::vector<Waypoint> &best_path, float &velocity, float &steering_angle_rate)
    {
        // DLOG(INFO) << "in calculateVelocityAndSteeringAngleRate:";
        std::vector<Waypoint> waypoint_vec;
        Waypoint car_pos;
        car_pos.x = current_state_in_map_frame_.x;
        car_pos.y = current_state_in_map_frame_.y;
        car_pos.heading = current_state_in_map_frame_.yaw;
        int closest_index = PlannerHelpers::getClosestNextWaypointIndex(best_path, car_pos);

        if (closest_index + 1 >= best_path.size())
            closest_index = best_path.size() - 2;
        DLOG_IF(FATAL, closest_index < 0) << "FATAL: closest_index smaller than zero!!!";
        DLOG_IF(FATAL, closest_index >= best_path.size()) << "FATAL: closest_index larger than best_path size!!!";
        // change to velocity planner
        waypoint_vec = velocity_planner_ptr_->planVelocity(best_path, current_state_in_map_frame_.speed);
        if ((closest_index + 1) < waypoint_vec.size())
        {
            velocity = waypoint_vec[closest_index + 1].speed;
            steering_angle_rate = waypoint_vec[closest_index].angular_speed;
            // DLOG(INFO) << "closest_index is " << closest_index << " velocity is " << velocity << " angular speed is " << steering_angle_rate;
        }
        else
        {
            velocity = waypoint_vec[closest_index].speed;
            steering_angle_rate = waypoint_vec[closest_index].angular_speed;
            // DLOG(INFO) << "closest_index is " << closest_index << " velocity is " << velocity << " angular speed is " << steering_angle_rate;
        }
        // DLOG(INFO) << "current vehicle speed is " << current_state_in_map_frame_.speed;

        for (const auto waypoint : waypoint_vec)
        {
            if (waypoint.speed > params_.max_linear_velocity || waypoint.speed < 1e-3)
            {
                for (const auto waypoint : waypoint_vec)
                {
                    // DLOG(INFO) << "velocity is " << waypoint.speed;
                }
            }
        }
        // plot(waypoint_vec);
        // velocity = best_path[closest_index].speed;
        // steering_angle_rate = calculateAngleVelocity(best_path[closest_index], best_path[closest_index + 1]);
        // DLOG(INFO) << "out calculateVelocityAndSteeringAngleRate.";
        return waypoint_vec;
    }

    void AdaptiveOpenLocalPlannerROS::goalCheck()
    {
        // Transform global plan to the frame of interest (w.r.t. the local costmap)
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        int goal_idx;
        geometry_msgs::PoseStamped robot_pose;
        costmap_ros_->getRobotPose(robot_pose);

        geometry_msgs::TransformStamped tf_plan_to_global;
        transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, params_.max_global_plan_lookahead_dist,
                            transformed_plan, &goal_idx, &tf_plan_to_global);

        // check if global goal is reached
        geometry_msgs::PoseStamped global_goal;
        tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
        double dx = global_goal.pose.position.x - robot_pose_.x();
        double dy = global_goal.pose.position.y - robot_pose_.y();
        double delta_orient = g2o::normalize_theta(tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta());

        if (fabs(std::sqrt(dx * dx + dy * dy)) < params_.xy_goal_tolerance && fabs(delta_orient) < params_.yaw_goal_tolerance)
        {
            goal_reached_ = true;
            // DLOG(INFO) << "Goal reached! current pose is " << robot_pose_.x() << " " << robot_pose_.y() << " " << robot_pose_.theta() << " goal pose is " << global_goal.pose.position.x << " " << global_goal.pose.position.y << " " << tf2::getYaw(global_goal.pose.orientation);
        }
        else
        {
            // DLOG(INFO) << "Goal not reached! current pose is " << robot_pose_.x() << " " << robot_pose_.y() << " " << robot_pose_.theta() << " goal pose is " << global_goal.pose.position.x << " " << global_goal.pose.position.y << " " << tf2::getYaw(global_goal.pose.orientation);
        }
    }

    bool AdaptiveOpenLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
    {
        if (global_plan.empty())
            return true;

        try
        {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform);

            double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end())
            {
                double dx = robot.pose.position.x - it->pose.position.x;
                double dy = robot.pose.position.y - it->pose.position.y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < dist_thresh_sq)
                {
                    erase_end = it;
                    break;
                }
                ++it;
            }
            if (erase_end == global_plan.end())
                return false;

            if (erase_end != global_plan.begin())
                global_plan.erase(global_plan.begin(), erase_end);
        }
        catch (const tf::TransformException &ex)
        {
            ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }
        return true;
    }

    bool AdaptiveOpenLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan, const geometry_msgs::PoseStamped &global_pose, const costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped> &transformed_plan, int *current_goal_idx, geometry_msgs::TransformStamped *tf_plan_to_global) const
    {
        // this method is a slightly modified version of base_local_planner/goal_functions.h

        const geometry_msgs::PoseStamped &plan_pose = global_plan[0];

        transformed_plan.clear();

        try
        {
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(params_.transform_tolerance));

            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

            // we'll discard points on the plan that are outside the local costmap
            double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                             costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                                    // located on the border of the local costmap

            int i = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = 1e10;

            // we need to loop to a point on the plan that is within a certain distance of the robot
            for (int j = 0; j < (int)global_plan.size(); ++j)
            {
                double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
                double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
                if (new_sq_dist > sq_dist_threshold)
                    break; // force stop if we have reached the costmap border

                if (new_sq_dist < sq_dist) // find closest distance
                {
                    sq_dist = new_sq_dist;
                    i = j;
                }
            }

            geometry_msgs::PoseStamped newer_pose;

            double plan_length = 0; // check cumulative Euclidean distance along the plan

            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped &pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_global_transform);

                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                // calculate distance to previous pose
                if (i > 0 && max_plan_length > 0)
                    plan_length += distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

                ++i;
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

                transformed_plan.push_back(newer_pose);

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx)
                    *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
            }

            // Return the transformation from the global plan to the global planning frame if desired
            if (tf_plan_to_global)
                *tf_plan_to_global = plan_to_global_transform;
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }

    void AdaptiveOpenLocalPlannerROS::calculateLinearVelocity(std::vector<Waypoint> &best_path)
    {
        for (uint i = 0; i < best_path.size() - 1; i++)
        {
            best_path[i].speed = calculateLinearVelocity(best_path[i], best_path[i + 1]);
        }
        // set last point speed to zero
        best_path.back().speed = 0;
    }

    double AdaptiveOpenLocalPlannerROS::calculateLinearVelocity(const Waypoint &current_point, const Waypoint &next_point)
    {
        double velocity;
        float dt = 2, distance;
        distance = distance2points(current_point, next_point);
        velocity = distance / dt;
        return velocity;
    }

    double AdaptiveOpenLocalPlannerROS::calculateAngleVelocity(const Waypoint &current_point, const Waypoint &next_point)
    {
        double angle_velocity = 0;

        // set dt to a constant value
        float dt = 0.55, angle_diff;
        angle_diff = next_point.heading - current_point.heading;
        if (angle_diff > M_PI_2 || angle_diff < -M_PI_2)
        {
            DLOG(WARNING) << "angle diff larger than M_PI_2 or smaller than M_PI_2, angle diff is " << angle_diff;
        }
        // DLOG(INFO) << "next point heading is " << next_point.heading << " current point heading is " << current_point.heading;
        angle_velocity = angle_diff / dt;
        return angle_velocity;
    }

    void AdaptiveOpenLocalPlannerROS::plot(const std::vector<Waypoint> &waypoint_vec)
    {
        // DLOG(INFO) << "in plot";
        // plot velocity
        matplotlibcpp::figure();
        // matplotlibcpp::ion();
        matplotlibcpp::clf();
        std::vector<std::pair<float, float>> linear_velocity_pair_vec = getLinearVelocityVec(waypoint_vec);
        std::vector<std::pair<float, float>> jerk_pair_vec = getJerk(waypoint_vec);
        // DLOG(INFO) << "jerk vec size is " << jerk_pair_vec.size();
        std::vector<std::string> title_vec = {"linear velocity", "jerk"};
        for (size_t i = 0; i < title_vec.size(); i++)
        {
            matplotlibcpp::subplot(2, 1, i + 1);
            std::vector<float> y_vec, x_vec;
            std::vector<std::pair<float, float>> target_pair_vec;
            if (title_vec[i] == "linear velocity")
            {
                target_pair_vec = linear_velocity_pair_vec;
            }
            if (title_vec[i] == "jerk")
            {
                target_pair_vec = jerk_pair_vec;
            }
            y_vec = PlannerHelpers::getSecondVec(target_pair_vec);
            x_vec = PlannerHelpers::getFirstVec(target_pair_vec);
            matplotlibcpp::plot(x_vec, y_vec, {{"label", "raw path"}});

            matplotlibcpp::legend({{"loc", "upper right"}});

            matplotlibcpp::title(title_vec[i]);
            matplotlibcpp::ylabel(title_vec[i]);
            matplotlibcpp::xlabel("distance[m]");
            // matplotlibcpp::ylim(0, 1);
            matplotlibcpp::grid(true);
            // matplotlibcpp::show();
            std::string path = "/home/jialiang/Code/thesis_ws/src/adaptive_open_local_planner/figs/";
            std::string file_name = "linear_velocity";

            auto now = std::time(0);
            std::string time_mark = std::to_string(now);
            matplotlibcpp::save(path + file_name + time_mark + ".png");
        }
        // DLOG(INFO) << "out plot";
    }

    std::vector<std::pair<float, float>> AdaptiveOpenLocalPlannerROS::getLinearVelocityVec(const std::vector<Waypoint> &waypoint_vec)
    {
        // DLOG(INFO) << "in getLinearVelocityVec";
        std::vector<std::pair<float, float>> linear_velocity_vec;
        float distance = 0, velocity;
        int i = 0;
        Waypoint start_point = waypoint_vec[0];
        for (const auto &element : waypoint_vec)
        {
            velocity = element.speed;
            if (i != 0)
            {
                distance = PlannerHelpers::getDistance(PlannerHelpers::extractVector(waypoint_vec, 0, i));
            }

            linear_velocity_vec.emplace_back(std::make_pair(distance, velocity));
            i++;
        }
        // DLOG(INFO) << "out getLinearVelocityVec";
        return linear_velocity_vec;
    }

    std::vector<std::pair<float, float>> AdaptiveOpenLocalPlannerROS::getJerk(const std::vector<Waypoint> &waypoint_vec)
    {
        // DLOG(INFO) << "in getJerk";
        std::vector<std::pair<float, float>> jerk_vec = velocity_planner_ptr_->findJerk();
        // DLOG(INFO) << "out getJerk";
        return jerk_vec;
    }

    std::vector<Eigen::Vector4d> AdaptiveOpenLocalPlannerROS::convertTrajectory(const std::vector<Waypoint> &waypoint_vec)
    {
        std::vector<Eigen::Vector4d> vec;
        Eigen::Vector4d waypoint;
        for (const auto &element : waypoint_vec)
        {
            waypoint[0] = element.x;
            waypoint[1] = element.y;
            waypoint[2] = element.heading;
            waypoint[3] = element.speed;
            vec.emplace_back(waypoint);
        }

        return vec;
    }

    std::vector<Eigen::Vector4d> AdaptiveOpenLocalPlannerROS::fakeTrajectory(const Eigen::Vector4d &current_state)
    {
        // use current state to cut off the fake trajectory
        std::vector<Eigen::Vector4d> vec;
        Eigen::Vector4d point;
        float x, y, heading, velocity;
        for (size_t i = 0; i < 20; i++)
        {
            x = 5.7;
            y = 0.8 + 0.1 * i;
            heading = 1.57;
            velocity = 1;
            if (y < current_state(1))
            {
                continue;
            }

            point << x, y, heading, velocity;
            vec.emplace_back(point);
        }
        for (size_t i = 0; i < 5; i++)
        {
            x = 5.7;
            y = 2.8 + 0.1 * i;
            heading = 1.57;
            velocity = 1 - 0.25 * i;
            if (y < current_state(1))
            {
                continue;
            }

            point << x, y, heading, velocity;
            vec.emplace_back(point);
        }

        // DLOG(INFO) << "current position is " << current_state[0] << " " << current_state[1] << " heading is " << current_state[2] << " speed is " << current_state[3];
        // for (const auto &element : vec)
        // {
        //     DLOG(INFO) << "ref element is " << element[0] << " " << element[1] << " heading is " << element[2] << " speed is " << element[3];
        // }
        return vec;
    }

    bool AdaptiveOpenLocalPlannerROS::goalCheck(const std::vector<Eigen::Vector4d> &trajectory)
    {

        double dx = trajectory.back()[0] - robot_pose_.x();
        double dy = trajectory.back()[1] - robot_pose_.y();
        double delta_orient = g2o::normalize_theta(trajectory.back()[2] - robot_pose_.theta());
        DLOG(INFO) << "current pose is " << robot_pose_.x() << " " << robot_pose_.y() << " " << robot_pose_.theta() << " goal is " << trajectory.back()[0] << " " << trajectory.back()[1] << " " << trajectory.back()[2];
        if (fabs(std::sqrt(dx * dx + dy * dy)) < params_.xy_goal_tolerance && fabs(delta_orient) < params_.yaw_goal_tolerance)
        {
            goal_reached_ = true;
            DLOG(INFO) << "Goal reached! current pose is " << robot_pose_.x() << " " << robot_pose_.y() << " " << robot_pose_.theta();
            return true;
        }
        else
        {
            // DLOG(INFO) << "Goal not reached! current pose is " << robot_pose_.x() << " " << robot_pose_.y() << " " << robot_pose_.theta() << " goal pose is " << global_goal.pose.position.x << " " << global_goal.pose.position.y << " " << tf2::getYaw(global_goal.pose.orientation);
            return false;
        }
    }

} // end namespace adaptive_open_local_planner