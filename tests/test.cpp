
#include "matplotlibcpp.h"
#include "mpc.h"
#include "path_evaluator.h"
std::vector<Eigen::Vector4d> fakeTrajectory()
{
    std::vector<Eigen::Vector4d> vec;
    Eigen::Vector4d point;
    float x, y, heading, velocity;
    for (size_t i = 0; i < 100; i++)
    {
        x = 5.5;
        y = 1 + 0.1 * i;
        heading = 1.57;
        velocity = 1;
        point << x, y, heading, velocity;
        vec.emplace_back(point);
    }
    return vec;
}

void Plot(const std::vector<Eigen::Vector4d> &path1, const Eigen::MatrixXd &path2)
{
    DLOG(INFO) << "Plot";
    matplotlibcpp::ion();
    matplotlibcpp::clf();
    // DLOG(INFO) << "size of path1 is " << path1.size() << " path2 is " << path2.cols();
    std::vector<float> x1, y1, x2, y2, heading1, heading2, velocity1, velocity2;
    // path
    matplotlibcpp::subplot(2, 2, 1);
    // int min_size = std::min(path1.size(), path2.cols());
    for (int i = 0; i < path2.cols(); i++)
    {
        x1.emplace_back(path1[i](1));
        y1.emplace_back(path1[i](0));
        heading1.emplace_back(path1[i](2));
        velocity1.emplace_back(path1[i](3));
        x2.emplace_back(path2.col(i)(1));
        y2.emplace_back(path2.col(i)(0));
        heading2.emplace_back(path2.col(i)(2));
        velocity2.emplace_back(path2.col(i)(3));
        DLOG(INFO) << "ref path is " << x1[i] << " " << y1[i] << " " << path1[i](2) << " " << path1[i](3);
        DLOG(INFO) << "mpc path is " << x2[i] << " " << y2[i] << " " << path2.col(i)(2) << " " << path2.col(i)(3);
    }
    // DLOG(INFO) << "size of x1 is " << x1.size() << " y1 is " << y1.size() << " x2 size is " << x2.size() << " y2 size is " << y2.size();
    matplotlibcpp::plot(x1, y1, {{"label", "ref path"}});
    matplotlibcpp::plot(x2, y2, {{"label", "mpc path"}});
    matplotlibcpp::ylim(0, 10);
    matplotlibcpp::legend({{"loc", "upper right"}});

    matplotlibcpp::title("path");
    matplotlibcpp::grid(true);

    matplotlibcpp::pause(0.1);
    // heading
    matplotlibcpp::subplot(2, 2, 2);
    matplotlibcpp::plot(heading1, {{"label", "ref path"}});
    matplotlibcpp::plot(heading2, {{"label", "mpc path"}});
    matplotlibcpp::ylim(-3.14, 3.14);
    matplotlibcpp::legend({{"loc", "upper right"}});
    matplotlibcpp::title("heading");
    matplotlibcpp::grid(true);
    // velocity
    matplotlibcpp::subplot(2, 2, 3);
    matplotlibcpp::plot(velocity1, {{"label", "ref path"}});
    matplotlibcpp::plot(velocity2, {{"label", "mpc path"}});
    matplotlibcpp::ylim(0.0, 1.5);
    matplotlibcpp::legend({{"loc", "upper right"}});
    matplotlibcpp::title("velocity");
    matplotlibcpp::grid(true);
    std::string file_name = "path";

    std::string path = "/home/jialiang/Code/thesis_ws/src/adaptive_open_local_planner/tests/";

    auto now = std::time(0);
    std::string time_mark = std::to_string(now);

    std::filesystem::create_directory(path);
    matplotlibcpp::save(path + file_name + time_mark + ".png");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");

    std::string path_topic, cmd_topic, jerk_topic, cost_topic, position_error_topic, heading_error_topic, velocity_error_topic, ackermann_cmd_topic;
    path_topic = "extracted_path_rviz";
    cmd_topic = "cmd_vel";
    jerk_topic = "jerk";
    cost_topic = "cost";
    position_error_topic = "position_error";
    heading_error_topic = "heading_error";
    velocity_error_topic = "velocity_error";
    ackermann_cmd_topic = "ackermann";

    adaptive_open_local_planner::MPC mpc_;

    double control_delay = 1;
    int planning_frequency = 10;
    double max_steer_angle = 10;
    double wheelbase_length = 1.006;
    // speed limit
    double max_linear_velocity = 5;
    double max_angular_acceleration = 100;
    double max_linear_acceleration = 4;
    int predicted_length = 20;
    double heading_weighting = 1;
    double last_heading_weighting = 1;
    double speed_weighting = 10;
    // DLOG(INFO) << "1 / planning_frequency is " << 1.0 / planning_frequency;
    mpc_.initialize(wheelbase_length, 1.0 / planning_frequency, control_delay, predicted_length, heading_weighting, last_heading_weighting, speed_weighting, max_linear_velocity, max_linear_acceleration, max_steer_angle, max_angular_acceleration, false);

    std::vector<Eigen::Vector4d> trajectory = fakeTrajectory();
    mpc_.inputRefTrajectory(trajectory);
    Eigen::Vector4d current_state(5, 1, 1.57, 0);
    Eigen::Vector2d control_vec = mpc_.output(current_state);
    Eigen::MatrixXd predictedState = mpc_.getPredictedState();

    Plot(trajectory, predictedState);

    return 0;
}
