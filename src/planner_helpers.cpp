#include "planner_helpers.h"
namespace adaptive_open_local_planner
{
    // checked it`s correct.
    int PlannerHelpers::getClosestNextWaypointIndex(const std::vector<Waypoint> &path, const Waypoint &current_pos)
    {
        double d = 0, min_d = DBL_MAX;
        int min_index = 0;
        for (int i = 0; i < path.size(); i++)
        {

            d = distance2pointsSqr(path[i], current_pos);
            bool ignore_angle_diff = true;
            if (ignore_angle_diff)
            {
                if (d < min_d)
                {
                    min_index = i;
                    min_d = d;
                }
                // DLOG_IF(INFO, path[i].heading != 0) << "index is " << i << " current point in global path is " << path[i].x << " " << path[i].y << " " << path[i].heading << " distance is " << d;
                // DLOG(INFO) << "index is " << i << " current point in global path is " << path[i].x << " " << path[i].y << " " << path[i].heading << " distance is " << d;
            }
            else
            {
                double angle_diff = angleBetweenTwoAnglesPositive(path[i].heading, current_pos.heading) * RAD2DEG;

                if (d < min_d && angle_diff < 45)
                {
                    min_index = i;
                    min_d = d;
                }
                DLOG(INFO) << "index is " << i << " current point in global path is " << path[i].x << " " << path[i].y << " " << path[i].heading << " distance is " << d << " angle diff is " << angle_diff;
            }
        }
        // DLOG_IF(INFO, min_index != 0) << "current pos is " << current_pos.x << " " << current_pos.y << " " << current_pos.heading << " Closest Global Waypoint Index = " << min_index;
        // DLOG(INFO) << "current pos is " << current_pos.x << " " << current_pos.y << " " << current_pos.heading << " Closest Global Waypoint Index = " << min_index;

        return min_index;
    }

    double PlannerHelpers::angleBetweenTwoAnglesPositive(const double &a1, const double &a2)
    {
        double diff = a1 - a2;
        if (diff < 0)
            diff = a2 - a1;

        if (diff > M_PI)
            diff = 2.0 * M_PI - diff;

        return diff;
    }

    double PlannerHelpers::fixNegativeAngle(const double &a)
    {
        double angle = 0;
        if (a < -2.0 * M_PI || a >= 2.0 * M_PI)
        {
            angle = fmod(a, 2.0 * M_PI);
        }
        else
        {
            angle = a;
        }

        if (angle < 0)
        {
            angle = 2.0 * M_PI + angle;
        }

        return angle;
    }

    void PlannerHelpers::fixPathDensity(std::vector<Waypoint> &path, const double &path_density)
    {
        if (path.size() == 0)
        {
            DLOG(WARNING) << "path size is zero!!!!";
            return;
        }

        if (path_density == 0)
        {
            DLOG(WARNING) << "Parameter: path_density is zero!!!!";
            return;
        }

        double distance = 0, angle = 0, margin = path_density * 0.01, remaining = 0;
        int num_points = 0;
        std::vector<Waypoint> fixed_path;
        fixed_path.push_back(path[0]);
        for (int start_index = 0, end_index = 1; end_index < path.size();)
        {
            distance += hypot(path[end_index].x - path[end_index - 1].x, path[end_index].y - path[end_index - 1].y) + remaining;
            angle = atan2(path[end_index].y - path[start_index].y, path[end_index].x - path[start_index].x);

            if (distance < path_density - margin) // downsample
            {
                end_index++;
                remaining = 0;
            }
            else if (distance > (path_density + margin)) // upsample
            {
                Waypoint new_wp = path[start_index];
                num_points = distance / path_density;
                for (int k = 0; k < num_points; k++)
                {
                    new_wp.x = new_wp.x + path_density * cos(angle);
                    new_wp.y = new_wp.y + path_density * sin(angle);
                    fixed_path.push_back(new_wp);
                }
                remaining = distance - num_points * path_density;
                start_index++;
                path[start_index] = new_wp;
                distance = 0;
                end_index++;
            }
            else
            {
                distance = 0;
                remaining = 0;
                fixed_path.push_back(path[end_index]);
                end_index++;
                start_index = end_index - 1;
            }
        }
        if (fixed_path.back().x != path.back().x || fixed_path.back().y != path.back().y)
        {
            fixed_path.push_back(path.back());
        }
        path = fixed_path;
    }

    void PlannerHelpers::smoothPath(std::vector<Waypoint> &path, const double &smooth_tolerance, const double &smooth_data_weight, const double &smooth_weight)
    {
        if (path.size() <= 2)
            return;

        std::vector<Waypoint> smoothed_path = path;

        double change = smooth_tolerance;
        double xtemp, ytemp;
        int nIterations = 0;

        while (change >= smooth_tolerance)
        {
            change = 0.0;
            for (int i = 1; i < path.size() - 1; i++)
            {
                xtemp = smoothed_path[i].x;
                ytemp = smoothed_path[i].y;

                smoothed_path[i].x += smooth_data_weight * (path[i].x - smoothed_path[i].x);
                smoothed_path[i].y += smooth_data_weight * (path[i].y - smoothed_path[i].y);

                smoothed_path[i].x += smooth_weight * (smoothed_path[i - 1].x + smoothed_path[i + 1].x - (2.0 * smoothed_path[i].x));
                smoothed_path[i].y += smooth_weight * (smoothed_path[i - 1].y + smoothed_path[i + 1].y - (2.0 * smoothed_path[i].y));

                change += fabs(xtemp - smoothed_path[i].x);
                change += fabs(ytemp - smoothed_path[i].y);
            }
            nIterations++;
        }

        //   ROS_INFO("Number of iterations: %d", nIterations);

        path = smoothed_path;
    }

    void PlannerHelpers::calculateAngleAndCost(std::vector<Waypoint> &path, const double &prev_cost_)
    {
        if (path.size() < 2)
            return;

        if (path.size() == 2)
        {
            // path[0].heading = fixNegativeAngle(atan2(path[1].y - path[0].y, path[1].x - path[0].x));
            path[0].heading = atan2(path[1].y - path[0].y, path[1].x - path[0].x);
            path[0].cost = prev_cost_;
            path[1].heading = path[0].heading;
            path[1].cost = path[0].cost + distance2points(path[0], path[1]);
            return;
        }

        // path[0].heading = fixNegativeAngle(atan2(path[1].y - path[0].y, path[1].x - path[0].x));
        path[0].heading = atan2(path[1].y - path[0].y, path[1].x - path[0].x);
        path[0].cost = prev_cost_;

        for (int j = 1; j < path.size() - 1; j++)
        {
            // path[j].heading = fixNegativeAngle(atan2(path[j+1].y - path[j].y, path[j+1].x - path[j].x));
            path[j].heading = atan2(path[j + 1].y - path[j].y, path[j + 1].x - path[j].x);
            path[j].cost = path[j - 1].cost + distance2points(path[j - 1], path[j]);
        }

        int j = (int)path.size() - 1;

        path[j].heading = path[j - 1].heading;
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1], path[j]);

        for (int j = 0; j < path.size() - 1; j++)
        {
            if (path[j].x == path[j + 1].x && path[j].y == path[j + 1].y)
                path[j].heading = path[j + 1].heading;
        }

        return;
    }

    bool PlannerHelpers::getRelativeInfo(const std::vector<Waypoint> &path, const Waypoint &current_pos, RelativeInfo &info)
    {
        // DLOG(INFO) << "in getRelativeInfo:";
        if (path.size() < 2)
            return false;

        Waypoint p0, p1;
        if (path.size() == 2)
        {
            p0 = path[0];
            p1.x = (path[0].x + path[1].x) / 2.0;
            p1.y = (path[0].y + path[1].y) / 2.0;
            p1.heading = path[0].heading;
            info.front_index = 1;
            info.back_index = 0;
        }
        else
        {
            info.front_index = PlannerHelpers::getClosestNextWaypointIndex(path, current_pos);

            if (info.front_index > 0)
                info.back_index = info.front_index - 1;
            else
                info.back_index = 0;

            if (info.front_index == 0)
            {
                p0 = path[info.front_index];
                p1 = path[info.front_index + 1];
            }
            else if (info.front_index > 0 && info.front_index < path.size() - 1)
            {
                p0 = path[info.front_index - 1];
                p1 = path[info.front_index];
            }
            else
            {
                p0 = path[info.front_index - 1];
                p1.x = (p0.x + path[info.front_index].x) / 2.0;
                p1.y = (p0.y + path[info.front_index].y) / 2.0;
                p1.heading = p0.heading;
            }
        }

        Waypoint prevWP = p0;
        Mat3 rotationMat(-p1.heading);
        Mat3 translationMat(-current_pos.x, -current_pos.y);
        Mat3 invRotationMat(p1.heading);
        Mat3 invTranslationMat(current_pos.x, current_pos.y);

        p0 = translationMat * p0;
        p0 = rotationMat * p0;
        p1 = translationMat * p1;
        p1 = rotationMat * p1;

        double m = (p1.y - p0.y) / (p1.x - p0.x);
        info.perp_distance = p1.y - m * p1.x; // solve for x = 0
        // DLOG(INFO) << "m: " << m ;

        if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance))
            info.perp_distance = 0;

        info.to_front_distance = fabs(p1.x); // distance on the x axes

        info.perp_point = p1;
        info.perp_point.x = 0;                  // on the same y axis of the car
        info.perp_point.y = info.perp_distance; // perp distance between the car and the trajectory

        info.perp_point = invRotationMat * info.perp_point;
        info.perp_point = invTranslationMat * info.perp_point;

        info.from_back_distance = hypot(info.perp_point.y - prevWP.y, info.perp_point.x - prevWP.x);

        info.angle_diff = angleBetweenTwoAnglesPositive(p1.heading, current_pos.heading) * RAD2DEG;

        return true;
    }

    void PlannerHelpers::predictConstantTimeCostForTrajectory(std::vector<Waypoint> &path, const VehicleState &current_state_in_map_frame)
    {
        if (path.size() == 0)
            return;

        for (int i = 0; i < path.size(); i++)
            path[i].time_cost = -1;

        // TODO: less than ?? (min threshold)
        if (current_state_in_map_frame.speed < 0.1)
            return;

        RelativeInfo info;
        Waypoint car_pos;
        car_pos.x = current_state_in_map_frame.x;
        car_pos.y = current_state_in_map_frame.y;
        car_pos.heading = current_state_in_map_frame.yaw;
        getRelativeInfo(path, car_pos, info);

        double total_distance = 0;
        double accum_time = 0;

        path[info.front_index].time_cost = 0;
        if (info.front_index == 0)
            info.front_index++;

        for (int i = info.front_index; i < path.size(); i++)
        {
            total_distance += hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
            accum_time = total_distance / current_state_in_map_frame.speed;
            path[i].time_cost = accum_time;
        }
    }

    double PlannerHelpers::getExactDistanceOnTrajectory(const std::vector<Waypoint> &trajectory, const RelativeInfo &p1, const RelativeInfo &p2)
    {
        if (trajectory.size() == 0)
            return 0;

        if (p2.front_index == p1.front_index && p2.back_index == p1.back_index)
        {
            return p2.to_front_distance - p1.to_front_distance;
        }
        else if (p2.back_index >= p1.front_index)
        {
            double d_on_path = p1.to_front_distance + p2.from_back_distance;
            for (int i = p1.front_index; i < p2.back_index; i++)
                d_on_path += hypot(trajectory[i + 1].y - trajectory[i].y, trajectory[i + 1].x - trajectory[i].x);

            return d_on_path;
        }
        else if (p2.front_index <= p1.back_index)
        {
            double d_on_path = p1.from_back_distance + p2.to_front_distance;
            for (int i = p2.front_index; i < p1.back_index; i++)
                d_on_path += hypot(trajectory[i + 1].y - trajectory[i].y, trajectory[i + 1].x - trajectory[i].x);

            return -d_on_path;
        }

        return 0;
    }

    double PlannerHelpers::checkTrajectoryForCollision(const std::vector<Waypoint> &trajectory, const std::vector<CircleObstacle> &circle_obstacles, const std::vector<BoxObstacle> &box_obstacles,
                                                       const double &safety_radius, const double &vehicle_width, const double &vehicle_length, const double &wheelbase_length,
                                                       const double &horizontal_safety_distance, const double &vertical_safety_distance)
    {
        double closest_obs_distance = DBL_MAX;

        for (int it = 0; it < trajectory.size(); it++)
        {
            for (int io = 0; io < circle_obstacles.size(); io++)
            {
                // first stage of check
                double distance = hypot(trajectory[it].x - circle_obstacles[io].x, trajectory[it].y - circle_obstacles[io].y);
                distance = distance - safety_radius - circle_obstacles[io].radius;
                if (distance <= 0)
                {
                    // second stage of check
                    Mat3 rotationMat(trajectory[it].heading - M_PI_2);
                    Mat3 translationMat(trajectory[it].x, trajectory[it].y);

                    double critical_lateral_distance = vehicle_width / 2.0 + horizontal_safety_distance;
                    double critical_long_front_distance = wheelbase_length / 2.0 + vehicle_length / 2.0 + vertical_safety_distance;
                    double critical_long_back_distance = vehicle_length / 2.0 + vertical_safety_distance - wheelbase_length / 2.0;

                    Waypoint bottom_left;
                    bottom_left.x = -critical_lateral_distance;
                    bottom_left.y = -critical_long_back_distance;

                    Waypoint bottom_right;
                    bottom_right.x = critical_lateral_distance;
                    bottom_right.y = -critical_long_back_distance;

                    Waypoint top_right;
                    top_right.x = critical_lateral_distance;
                    top_right.y = critical_long_front_distance;

                    Waypoint top_left;
                    top_left.x = -critical_lateral_distance;
                    top_left.y = critical_long_front_distance;

                    bottom_left = rotationMat * bottom_left;
                    bottom_left = translationMat * bottom_left;

                    top_right = rotationMat * top_right;
                    top_right = translationMat * top_right;

                    bottom_right = rotationMat * bottom_right;
                    bottom_right = translationMat * bottom_right;

                    top_left = rotationMat * top_left;
                    top_left = translationMat * top_left;

                    Polygon car_polygon;
                    car_polygon.points.push_back(bottom_left);
                    car_polygon.points.push_back(bottom_right);
                    car_polygon.points.push_back(top_right);
                    car_polygon.points.push_back(top_left);
                    car_polygon.points.push_back(bottom_left);

                    if (car_polygon.BoxInsidePolygon(car_polygon, box_obstacles[io]))
                        return -1; // collision
                }
                if (fabs(distance) < closest_obs_distance)
                    closest_obs_distance = fabs(distance);
            }
        }
        return closest_obs_distance;
    }

    void PlannerHelpers::calculateTransitionCosts(std::vector<PathCost> &trajectory_costs, const int &curr_trajectory_index, const double &roll_out_density)
    {
        for (int ic = 0; ic < trajectory_costs.size(); ic++)
        {
            trajectory_costs[ic].transition_cost = fabs(roll_out_density * (ic - curr_trajectory_index));
        }
    }

    void PlannerHelpers::calculateLateralAndLongitudinalCostsStatic(std::vector<PathCost> &trajectory_costs, const std::vector<std::vector<Waypoint>> &roll_outs,
                                                                    const std::vector<Waypoint> &extracted_path, std::vector<Waypoint> &contour_points,
                                                                    const VehicleState &current_state_in_map_frame, visualization_msgs::Marker &car_footprint_marker,
                                                                    visualization_msgs::Marker &safety_box_marker,
                                                                    const double &vehicle_length, const double &vehicle_width,
                                                                    const double &wheelbase_length, const double &horizontal_safety_distance,
                                                                    const double &vertical_safety_distance, const double &max_steer_angle,
                                                                    const double &min_following_distance, const double &lateral_skip_distance)
    {
        double critical_lateral_distance = vehicle_width / 2.0 + horizontal_safety_distance;
        double critical_long_front_distance = wheelbase_length / 2.0 + vehicle_length / 2.0 + vertical_safety_distance;
        double critical_long_back_distance = vehicle_length / 2.0 + vertical_safety_distance - wheelbase_length / 2.0;

        Mat3 invRotationMat(current_state_in_map_frame.yaw - M_PI_2);
        Mat3 invTranslationMat(current_state_in_map_frame.x, current_state_in_map_frame.y);

        double corner_slide_distance = critical_lateral_distance / 2.0;
        double ratio_to_angle = corner_slide_distance / max_steer_angle;
        double slide_distance = current_state_in_map_frame.steer * ratio_to_angle;

        Waypoint bottom_left;
        bottom_left.x = -vehicle_width / 2.0;
        bottom_left.y = -vehicle_length / 2.0 + wheelbase_length / 2.0;

        Waypoint bottom_right;
        bottom_right.x = vehicle_width / 2.0;
        bottom_right.y = -vehicle_length / 2.0 + wheelbase_length / 2.0;

        Waypoint top_right_car;
        top_right_car.x = vehicle_width / 2.0;
        top_right_car.y = wheelbase_length / 2.0 + vehicle_length / 2.0;

        Waypoint top_left_car;
        top_left_car.x = -vehicle_width / 2.0;
        top_left_car.y = wheelbase_length / 2.0 + vehicle_length / 2.0;

        bottom_left = invRotationMat * bottom_left;
        bottom_left = invTranslationMat * bottom_left;

        bottom_right = invRotationMat * bottom_right;
        bottom_right = invTranslationMat * bottom_right;

        top_right_car = invRotationMat * top_right_car;
        top_right_car = invTranslationMat * top_right_car;

        top_left_car = invRotationMat * top_left_car;
        top_left_car = invTranslationMat * top_left_car;

        std::vector<Waypoint> car_footprint;
        car_footprint.push_back(bottom_left);
        car_footprint.push_back(bottom_right);
        car_footprint.push_back(top_right_car);
        car_footprint.push_back(top_left_car);
        car_footprint.push_back(bottom_left);

        VisualizationHelpers::createCarFootprintMarker(car_footprint, car_footprint_marker);

        bottom_left.x = -critical_lateral_distance;
        bottom_left.y = -critical_long_back_distance;
        bottom_left.heading = 0;

        bottom_right.x = critical_lateral_distance;
        bottom_right.y = -critical_long_back_distance;
        bottom_right.heading = 0;

        top_right_car.x = critical_lateral_distance;
        top_right_car.y = wheelbase_length / 3.0 + vehicle_length / 3.0;
        top_right_car.heading = 0;

        top_left_car.x = -critical_lateral_distance;
        top_left_car.y = wheelbase_length / 3.0 + vehicle_length / 3.0;
        top_left_car.heading = 0;

        Waypoint top_right;
        top_right.x = critical_lateral_distance - slide_distance;
        top_right.y = critical_long_front_distance;
        top_right.heading = 0;

        Waypoint top_left;
        top_left.x = -critical_lateral_distance - slide_distance;
        top_left.y = critical_long_front_distance;
        top_left.heading = 0;

        bottom_left = invRotationMat * bottom_left;
        bottom_left = invTranslationMat * bottom_left;

        top_right = invRotationMat * top_right;
        top_right = invTranslationMat * top_right;

        bottom_right = invRotationMat * bottom_right;
        bottom_right = invTranslationMat * bottom_right;

        top_left = invRotationMat * top_left;
        top_left = invTranslationMat * top_left;

        top_right_car = invRotationMat * top_right_car;
        top_right_car = invTranslationMat * top_right_car;

        top_left_car = invRotationMat * top_left_car;
        top_left_car = invTranslationMat * top_left_car;

        Polygon safety_box;
        safety_box.points.push_back(bottom_left);
        safety_box.points.push_back(bottom_right);
        safety_box.points.push_back(top_right_car);
        safety_box.points.push_back(top_right);
        safety_box.points.push_back(top_left);
        safety_box.points.push_back(top_left_car);

        VisualizationHelpers::createSafetyBoxMarker(safety_box.points, safety_box_marker);

        if (roll_outs.size() > 0 && roll_outs[0].size() > 0)
        {
            RelativeInfo car_info;
            Waypoint car_pos;
            car_pos.x = current_state_in_map_frame.x;
            car_pos.y = current_state_in_map_frame.y;
            car_pos.heading = current_state_in_map_frame.yaw;
            // DLOG(INFO) << "Car heading: " << car_pos.heading ;
            getRelativeInfo(extracted_path, car_pos, car_info);

            for (int it = 0; it < roll_outs.size(); it++)
            {
                trajectory_costs[it].lateral_cost = 0;
                trajectory_costs[it].longitudinal_cost = 0;
                // int skip_id = -1;
                for (int ic = 0; ic < contour_points.size(); ic++)
                {
                    //    if(skip_id == contour_points[icon].id) continue;
                    // DLOG(INFO) << "--> Checking obstacles against roll out no: " << it ;
                    RelativeInfo obj_info;
                    contour_points[ic].heading = car_pos.heading;
                    getRelativeInfo(extracted_path, contour_points[ic], obj_info);
                    double longitudinalDist = getExactDistanceOnTrajectory(extracted_path, car_info, obj_info);
                    // DLOG(INFO) << "Car_info front_index: " << car_info.front_index ;
                    // DLOG(INFO) << "Obj_info front_index: " << obj_info.front_index ;
                    // DLOG(INFO) << "Longitudinal distance: " << longitudinalDist ;
                    if (obj_info.front_index == 0 && longitudinalDist > 0)
                        longitudinalDist = -longitudinalDist;
                    // DLOG(INFO) << "Longitudinal distance: " << longitudinalDist ;

                    // double close_in_percentage = 1;
                    // close_in_percentage = ((longitudinalDist- critical_long_front_distance)/params.rollInMargin)*4.0;

                    // if(close_in_percentage <= 0 || close_in_percentage > 1) close_in_percentage = 1;

                    double distance_from_center = trajectory_costs[it].distance_from_center;

                    // if(close_in_percentage < 1)
                    //     distance_from_center = distance_from_center - distance_from_center * (1.0 - close_in_percentage);

                    double lateralDist = fabs(obj_info.perp_distance - distance_from_center);
                    // DLOG(INFO) << "Lateral distance: " << lateralDist ;

                    longitudinalDist = longitudinalDist - critical_long_front_distance;
                    // DLOG(INFO) << "Longitudinal distance: " << longitudinalDist ;

                    if (longitudinalDist < -vehicle_length || longitudinalDist > min_following_distance || lateralDist > lateral_skip_distance)
                    {
                        continue;
                    }

                    if (safety_box.PointInsidePolygon(safety_box, contour_points[ic]) == true)
                    {
                        // DLOG(INFO) << "Point inside polygon!!" ;
                        trajectory_costs[it].bBlocked = true;
                    }

                    if (lateralDist <= critical_lateral_distance && longitudinalDist >= -vehicle_length / 1.5 && longitudinalDist <= min_following_distance)
                    {
                        // DLOG(INFO) << "lateralDist: " << lateralDist ;
                        // DLOG(INFO) << "Critical lateral distance: " << critical_lateral_distance ;
                        // DLOG(INFO) << "longitudinalDist: " << longitudinalDist ;
                        trajectory_costs[it].bBlocked = true;
                    }

                    if (lateralDist != 0)
                        trajectory_costs[it].lateral_cost += 1.0 / lateralDist;

                    if (longitudinalDist != 0)
                        trajectory_costs[it].longitudinal_cost += 1.0 / fabs(longitudinalDist);

                    if (longitudinalDist >= -critical_long_front_distance && longitudinalDist < trajectory_costs[it].closest_obj_distance)
                    {
                        trajectory_costs[it].closest_obj_distance = longitudinalDist;
                        trajectory_costs[it].closest_obj_velocity = 0;
                    }
                }
            }
        }
    }

    void PlannerHelpers::calculateCurvatureCosts(std::vector<PathCost> &trajectory_costs, const std::vector<std::vector<Waypoint>> &roll_outs)
    {
        for (int i = 0; i < roll_outs.size(); i++)
        {
            trajectory_costs[i].curvature_cost = 0;

            for (int j = 0; j < roll_outs[i].size() - 1; j++)
            {
                trajectory_costs[i].curvature_cost += fabs(roll_outs[i][j + 1].heading - roll_outs[i][j].heading);
            }
        }
    }

    void PlannerHelpers::normalizeCosts(std::vector<PathCost> &trajectory_costs, const double &priority_weight,
                                        const double &transition_weight, const double &lat_weight, const double &long_weight, const double &curvature_weight)
    {
        // Normalize costs
        double totalPriorities = 0;
        // double totalChange = 0;
        double totalLateralCosts = 0;
        double totalLongitudinalCosts = 0;
        double transitionCosts = 0;
        double totalCurvatureCosts = 0;
        // double collisionCosts = 0;

        for (int ic = 0; ic < trajectory_costs.size(); ic++)
        {
            totalPriorities += trajectory_costs[ic].priority_cost;
            transitionCosts += trajectory_costs[ic].transition_cost;
            // collisionCosts += trajectory_costs[ic].closest_obj_cost;
        }

        for (int ic = 0; ic < trajectory_costs.size(); ic++)
        {
            // totalChange += trajectory_costs[ic].lane_change_cost;
            totalLateralCosts += trajectory_costs[ic].lateral_cost;
            totalLongitudinalCosts += trajectory_costs[ic].longitudinal_cost;
            totalCurvatureCosts += trajectory_costs[ic].curvature_cost;
        }

        //  cout << "------ Normalizing Step " << endl;
        for (int ic = 0; ic < trajectory_costs.size(); ic++)
        {
            if (totalPriorities != 0)
                trajectory_costs[ic].priority_cost = trajectory_costs[ic].priority_cost / totalPriorities;
            else
                trajectory_costs[ic].priority_cost = 0;

            if (transitionCosts != 0)
                trajectory_costs[ic].transition_cost = trajectory_costs[ic].transition_cost / transitionCosts;
            else
                trajectory_costs[ic].transition_cost = 0;

            // if(totalChange != 0)
            //     trajectory_costs[ic].lane_change_cost = trajectory_costs[ic].lane_change_cost / totalChange;
            // else
            //     trajectory_costs[ic].lane_change_cost = 0;

            if (totalLateralCosts != 0)
                trajectory_costs[ic].lateral_cost = trajectory_costs[ic].lateral_cost / totalLateralCosts;
            else
                trajectory_costs[ic].lateral_cost = 0;

            if (totalLongitudinalCosts != 0)
                trajectory_costs[ic].longitudinal_cost = trajectory_costs[ic].longitudinal_cost / totalLongitudinalCosts;
            else
                trajectory_costs[ic].longitudinal_cost = 0;

            // if(collisionCosts != 0)
            //     trajectory_costs[ic].closest_obj_cost = trajectory_costs[ic].closest_obj_cost / collisionCosts;
            // else
            //     trajectory_costs[ic].closest_obj_cost = 0;

            if (totalCurvatureCosts != 0)
                trajectory_costs[ic].curvature_cost = trajectory_costs[ic].curvature_cost / totalCurvatureCosts;
            else
                trajectory_costs[ic].curvature_cost = 0;

            trajectory_costs[ic].cost = (priority_weight * trajectory_costs[ic].priority_cost + transition_weight * trajectory_costs[ic].transition_cost + lat_weight * trajectory_costs[ic].lateral_cost + long_weight * trajectory_costs[ic].longitudinal_cost + curvature_weight * trajectory_costs[ic].curvature_cost) /
                                        (priority_weight + transition_weight + lat_weight + long_weight + curvature_weight);
            // trajectory_costs[ic].cost = (priority_weight*trajectory_costs[ic].priority_cost + transition_weight*trajectory_costs[ic].transition_cost + collision_weight*trajectory_costs[ic].closest_obj_cost)/3.0;

            // DLOG(INFO) << "Index: " << ic                       << ", Priority: " << trajectory_costs[ic].priority_cost                       << ", Transition: " << trajectory_costs[ic].transition_cost                       << ", Lat: " << trajectory_costs[ic].lateral_cost                       << ", Long: " << trajectory_costs[ic].longitudinal_cost                       //    << ", Change: " << trajectory_costs.at(ic).lane_change_cost                       //    << ", Collision: " << trajectory_costs[ic].closest_obj_cost                       << ", Curvature: " << trajectory_costs[ic].curvature_cost                       << ", Avg: " << trajectory_costs[ic].cost;
        }

        // DLOG(INFO) << "------------------------ ";
    }

    void PlannerHelpers::convert(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan, std::vector<Waypoint> &path)
    {
        for (const auto &item : orig_global_plan)
        {
            Waypoint point;
            point = convert(item);
            path.emplace_back(point);
            // DLOG(INFO) << "item in original global plan is " << item.pose.position.x << " " << item.pose.position.y << " " << item.pose.position.z << " orientation is " << tf::getYaw(item.pose.orientation);
        }
    }

    void PlannerHelpers::convert(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan, nav_msgs::Path &path)
    {
        path.header.frame_id = "map";
        for (const auto &item : orig_global_plan)
        {
            geometry_msgs::PoseStamped pose;
            pose = item;
            path.poses.emplace_back(pose);
        }
    }

    std::vector<Waypoint> PlannerHelpers::convert(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        std::vector<Waypoint> global_plan;
        convert(orig_global_plan, global_plan);
        return global_plan;
    }

    Waypoint PlannerHelpers::convert(const geometry_msgs::PoseStamped &pose)
    {
        Waypoint point;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        point.heading = tf::getYaw(pose.pose.orientation);
        return point;
    }

    std::vector<Eigen::Vector3f> PlannerHelpers::convert(const nav_msgs::Path::ConstPtr &path)
    {
        std::vector<Eigen::Vector3f> path_vec;

        for (uint i = 0; i < path->poses.size(); ++i)
        {
            Eigen::Vector3f point;
            point.x() = (path->poses[i].pose.position.x);
            point.y() = (path->poses[i].pose.position.y);
            // not sure this is correct;
            point.z() = tf::getYaw(path->poses[i].pose.orientation);

            path_vec.emplace_back(point);
        }

        return path_vec;
    }

    float PlannerHelpers::CalculateCurvature(const Eigen::Vector2f &pre, const Eigen::Vector2f &current, const Eigen::Vector2f &succ)
    {
        float curvature = 0;
        // get three points from path

        if (pre == current || current == succ)
        {
            LOG(WARNING) << "WARNING: In CalculateCurvature: some points are equal, skip these points for curvature calculation!! pre is " << pre.x() << " " << pre.y() << " current is " << current.x() << " " << current.y() << " succ is " << succ.x() << " " << succ.y();
            return curvature;
        }

        // get two vector between these three nodes
        Eigen::Vector2f pre_vector = current - pre;

        Eigen::Vector2f succ_vector = succ - current;

        // calculate delta distance and delta angle
        float delta_distance = succ_vector.norm();
        float pre_vector_length = pre_vector.norm();

        // there would some calculation error here causing number inside acos greater than 1 or smaller than -1.
        float delta_angle = std::acos(Clamp(pre_vector.dot(succ_vector) / (delta_distance * pre_vector_length), 1, -1));

        curvature = (delta_angle) / pre_vector_length;
        LOG_IF(INFO, curvature > 1000) << "current is: " << current(0, 0) << " y is: " << current.y() << " succ x is :" << succ(0, 0) << " y is: " << succ.y() << " pre x is :" << pre(0, 0) << " y is: " << pre.y() << " pre_vector x is :" << pre_vector(0, 0) << " y is: " << pre_vector.y() << " succ_vector x is :" << succ_vector(0, 0) << "y is: " << succ_vector.y() << " delta_distance is:" << delta_distance << " pre_vector_length is: " << pre_vector_length << " delta_angle is: " << delta_angle << " curvature is " << curvature;
        return curvature;
    }

    float PlannerHelpers::CalculateCurvature(const Waypoint &pre, const Waypoint &current)
    {
        // DLOG(INFO) << "in CalculateCurvature";
        float curvature = 0;
        // get three points from path

        if (pre == current)
        {
            LOG(WARNING) << "WARNING: In CalculateCurvature: some points are equal, skip these points for curvature calculation!! pre is " << pre.x << " " << pre.y << " current is " << current.x << " " << current.y;
            return curvature;
        }

        float delta_distance = getDistance(pre, current);

        float delta_angle = current.heading - pre.heading;

        curvature = (delta_angle) / delta_distance;
        DLOG_IF(INFO, std::isnan(curvature)) << "curvature is NAN. delta angle is " << delta_angle << " delta distance is " << delta_distance;
        return curvature;
    }

    float PlannerHelpers::Clamp(const float &number, const float &upper_bound,
                                const float &lower_bound)
    {
        return std::max(lower_bound, std::min(number, upper_bound));
    }

    std::vector<Waypoint> PlannerHelpers::extractVector(const std::vector<Waypoint> &base, int start_index, int end_index)
    {
        std::vector<Waypoint> slice(base.begin() + start_index, base.begin() + end_index + 1);
        return slice;
    }

    float PlannerHelpers::getDistance(const std::vector<Waypoint> &local_path)
    {
        // DLOG(INFO) << "in getDistance";
        float distance = 0;
        for (size_t i = 0; i < local_path.size() - 1; i++)
        {
            // DLOG(INFO) << "index is " << i;
            distance = distance + getDistance(local_path[i], local_path[i + 1]);
        }
        return distance;
    }

    float PlannerHelpers::getDistance(const Waypoint &prev, const Waypoint &current)
    {
        // DLOG(INFO) << "in getDistance";
        float distance = 0;
        distance = std::sqrt((prev.x - current.x) * (prev.x - current.x) + (prev.y - current.y) * (prev.y - current.y));
        // DLOG(INFO) << "end of getDistance";
        DLOG_IF(INFO, std::isnan(distance)) << "distance is NAN. previous point is " << prev.x << " " << prev.y << " current point is " << current.x << " " << current.y;
        return distance;
    }

}
