#include "adaptive_open_local_planner_ros.h"
#include <pluginlib/class_list_macros.h>

// register this planner both as a BaseLocalPlanner
PLUGINLIB_EXPORT_CLASS(adaptive_open_local_planner::AdaptiveOpenLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace adaptive_open_local_planner
{

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
        DLOG(INFO) << "creating adaptive_open_local_planner planner";
        initialize(name, tf, costmap_ros);
    }
    void AdaptiveOpenLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            // create Node Handle with name of plugin (as used in move_base for loading)
            // ros::NodeHandle nh("~/" + name);
            ros::NodeHandle nh;
            params_.loadRosParamFromNodeHandle(nh);

            // Subscribe & Advertise
            odom_sub = nh.subscribe(params_.odom_topic, 1, &AdaptiveOpenLocalPlannerROS::odomCallback, this);

            extracted_path_rviz_pub = nh.advertise<nav_msgs::Path>(params_.extracted_path_rviz_topic, 1, true);
            current_pose_rviz_pub = nh.advertise<geometry_msgs::PoseStamped>(params_.current_pose_rviz_topic, 1, true);
            roll_outs_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(params_.roll_outs_rviz_topic, 1, true);
            weighted_trajectories_rviz_pub = nh.advertise<visualization_msgs::MarkerArray>(params_.weighted_trajectories_rviz_topic, 1, true);
            safety_box_rviz_pub = nh.advertise<visualization_msgs::Marker>(params_.safety_box_rviz_topic, 1, true);
            car_footprint_rviz_pub = nh.advertise<visualization_msgs::Marker>(params_.car_footprint_rviz_topic, 1, true);
            box_obstacle_rviz_pub = nh.advertise<visualization_msgs::Marker>(params_.box_obstacle_rviz_topic, 1, true);

            global_path_received = false;
            b_vehicle_state = false;

            prev_cost = 0;

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

            DLOG(INFO) << "adaptive_open_local_planner plugin initialized.";
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
        global_path_received = true;
        PlannerHelpers::convert(global_plan_, global_path);
        // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
        // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

        // reset goal_reached_ flag
        goal_reached_ = false;

        return true;
    }

    bool AdaptiveOpenLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {

        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped cmd_vel_stamped;
        bool outcome = computeVelocityCommands(dummy_pose, cmd_vel_stamped);
        cmd_vel = cmd_vel_stamped.twist;
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

        convertObstacle();
        if (!global_path_received)
        {
            ROS_WARN("Global path not received!");
            return false;
        }
        if (!b_vehicle_state)
        {
            ROS_WARN("Odom data not received!");
            return false;
        }

        std::vector<Waypoint> extracted_path, best_path;
        extractGlobalPathSection(extracted_path);

        std::vector<std::vector<Waypoint>> roll_outs;
        generateRollOuts(extracted_path, roll_outs);

        std::vector<PathCost> trajectory_costs;
        doOneStepStatic(roll_outs, extracted_path, trajectory_costs, best_path);
        float velocity, steering_angle_rate;
        calculateVelocity(best_path, velocity, steering_angle_rate);
        cmd_vel.twist.linear.x = velocity;
        cmd_vel.twist.linear.y = 0;
        cmd_vel.twist.angular.z = steering_angle_rate;
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
        b_vehicle_state = true;
        // DLOG(INFO) << "odom received.";
        current_state_in_map_frame_.speed = odom_msg->twist.twist.linear.x;
        if (fabs(odom_msg->twist.twist.linear.x) > 0.25)
            current_state_in_map_frame_.steer = atan(params_.wheelbase_length * odom_msg->twist.twist.angular.z / odom_msg->twist.twist.linear.x);

        if (initialized_)
        {
            // Get robot pose
            geometry_msgs::PoseStamped robot_pose;
            costmap_ros_->getRobotPose(robot_pose);

            current_state_in_map_frame_.yaw = tf::getYaw(robot_pose.pose.orientation);

            current_state_in_map_frame_.x = robot_pose.pose.position.x;
            current_state_in_map_frame_.y = robot_pose.pose.position.y;

            VisualizationHelpers::createCurrentPoseMarker(current_state_in_map_frame_, robot_pose);
            current_pose_rviz_pub.publish(robot_pose);
        }
    }

    void AdaptiveOpenLocalPlannerROS::extractGlobalPathSection(std::vector<Waypoint> &extracted_path)
    {
        if (global_path.size() < 2)
            return;

        extracted_path.clear();

        Waypoint car_pos;
        car_pos.x = current_state_in_map_frame_.x;
        car_pos.y = current_state_in_map_frame_.y;
        car_pos.heading = current_state_in_map_frame_.yaw;
        int closest_index = PlannerHelpers::getClosestNextWaypointIndex(global_path, car_pos);

        if (closest_index + 1 >= global_path.size())
            closest_index = global_path.size() - 2;

        double d = 0;

        for (int i = closest_index; i < (int)global_path.size(); i++)
        {
            extracted_path.push_back(global_path[i]);
            if (i > 0)
                d += hypot(global_path[i].x - global_path[i - 1].x, global_path[i].y - global_path[i - 1].y);
            if (d > params_.max_local_plan_distance)
                break;
        }

        if (extracted_path.size() < 2)
        {
            ROS_WARN("Karcher Local Planner Node: Extracted Global Plan is too small, Size = %d", (int)extracted_path.size());
            return;
        }

        PlannerHelpers::fixPathDensity(extracted_path, params_.path_density);
        PlannerHelpers::smoothPath(extracted_path, params_.smooth_tolerance, params_.smooth_data_weight, params_.smooth_weight);
        PlannerHelpers::calculateAngleAndCost(extracted_path, prev_cost);

        nav_msgs::Path path;
        VisualizationHelpers::createExtractedPathMarker(extracted_path, path);
        extracted_path_rviz_pub.publish(path);
    }

    void AdaptiveOpenLocalPlannerROS::generateRollOuts(const std::vector<Waypoint> &path, std::vector<std::vector<Waypoint>> &roll_outs)
    {
        // std::cout << "path size: " << path.size() << std::endl;
        if (path.size() == 0)
            return;
        if (params_.max_local_plan_distance <= 0)
            return;
        roll_outs.clear();

        int i_limit_index = (params_.sampling_tip_margin / 0.3) / params_.path_density;
        if (i_limit_index >= path.size())
            i_limit_index = path.size() - 1;
        // std::cout << "i_limit_index: " << i_limit_index << std::endl;

        int closest_index;
        double initial_roll_in_distance;

        RelativeInfo info;
        Waypoint car_pos;
        car_pos.x = current_state_in_map_frame_.x;
        car_pos.y = current_state_in_map_frame_.y;
        car_pos.heading = current_state_in_map_frame_.yaw;
        PlannerHelpers::getRelativeInfo(path, car_pos, info);
        initial_roll_in_distance = info.perp_distance;
        // std::cout << "closest_index: " << closest_index << std::endl;
        // std::cout << "initial_roll_in_distance: " << initial_roll_in_distance << std::endl;

        double remaining_distance = 0;
        for (int i = 0; i < path.size() - 1; i++)
        {
            remaining_distance += distance2points(path[i], path[i + 1]);
        }
        // std::cout << "remaining_distance: " << remaining_distance << std::endl;

        // calculate the starting index
        double d_limit = 0;
        int start_index = 0;
        int end_index = 0;
        // int far_index = closest_index;

        // calculate end index
        double start_distance = params_.roll_in_speed_factor * current_state_in_map_frame_.speed + params_.roll_in_margin;
        if (start_distance > remaining_distance)
            start_distance = remaining_distance;
        // std::cout << "start_distance: " << start_distance << std::endl;

        d_limit = 0;
        for (int i = 0; i < path.size() - 1; i++)
        {
            d_limit += distance2points(path[i], path[i + 1]);

            if (d_limit >= start_distance)
            {
                end_index = i;
                break;
            }
        }
        // std::cout << "far_index: " << far_index << std::endl;

        int central_trajectory_index = params_.roll_outs_number / 2;
        std::vector<double> end_laterals;
        for (int i = 0; i < params_.roll_outs_number + 1; i++)
        {
            double end_roll_in_distance = params_.roll_out_density * (i - central_trajectory_index);
            end_laterals.push_back(end_roll_in_distance);
            // std::cout << "roll out num: " << i << ", end_roll_in_distance: " << end_roll_in_distance << std::endl;
        }

        // calculate the actual calculation starting index
        d_limit = 0;
        int smoothing_start_index = start_index;
        int smoothing_end_index = end_index;

        for (int i = smoothing_start_index; i < path.size(); i++)
        {
            if (i > 0)
                d_limit += distance2points(path[i], path[i - 1]);
            if (d_limit > params_.sampling_tip_margin)
                break;

            smoothing_start_index++;
        }

        d_limit = 0;
        for (int i = end_index; i < path.size(); i++)
        {
            if (i > 0)
                d_limit += distance2points(path[i], path[i - 1]);
            if (d_limit > params_.sampling_tip_margin)
                break;

            smoothing_end_index++;
        }
        // std::cout << "start_index: " << start_index << ", end_index: " << end_index << ", smoothing_start_index: "
        //             << smoothing_start_index << ", smoothing_end_index: " << smoothing_end_index << std::endl;

        int nSteps = end_index - smoothing_start_index;
        // std::cout << "nSteps: " << nSteps << std::endl;

        std::vector<double> inc_list;
        std::vector<double> inc_list_inc;
        for (int i = 0; i < params_.roll_outs_number + 1; i++)
        {
            double diff = end_laterals[i] - initial_roll_in_distance;
            // std::cout << "diff: " << diff << std::endl;
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

            if (d_limit > params_.max_local_plan_distance) // max_roll_distance)
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
            PlannerHelpers::calculateAngleAndCost(roll_outs[i], prev_cost);
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
        // std::cout <<  "Current Index: " << curr_index << std::endl;
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
                    // if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist)
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

    void AdaptiveOpenLocalPlannerROS::calculateVelocity(const std::vector<Waypoint> &best_path, float &velocity, float &steering_angle_rate)
    {
        // set dt to a constant value
        float dt = 1;
        float distance, angle_diff;
        distance = distance2points(best_path[0], best_path[1]);
        // TODO angle normalization is needed
        angle_diff = best_path[1].heading - best_path[0].heading;
        velocity = distance / dt;
        steering_angle_rate = angle_diff / dt;
    }
} // end namespace adaptive_open_local_planner