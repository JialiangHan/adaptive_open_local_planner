#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include "planner_helpers.h"
#include "struct_defs.h"
#include "matrix.h"
#include "polygon.h"
#include "visualization_helpers.h"
#include "obstacles.h"
#include <math.h>
#include <limits>
#include <vector>
#include "parameter_manager.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "pose_se2.h"
#include "path_evaluator.h"
#include "velocity_planner.h"
// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include "matplotlibcpp.h"
// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include "mpc.h"
#include <ctime>
namespace adaptive_open_local_planner
{

    /**
     * @class AdaptiveOpenLocalPlannerROS
     * @brief Implements both nav_core::BaseLocalPlanner
     * interfaces, so the adaptive_open_local_planner plugin can be used both in move_base and move_base_flex (MBF).
     * @todo Escape behavior, more efficient obstacle handling
     */
    class AdaptiveOpenLocalPlannerROS : public nav_core::BaseLocalPlanner
    {
    public:
        /**
         * @brief Default constructor of the teb plugin
         */
        AdaptiveOpenLocalPlannerROS();
        AdaptiveOpenLocalPlannerROS(std::string name, tf2_ros::Buffer *tf,
                                    costmap_2d::Costmap2DROS *costmap_ros);
        /**
         * @brief  Destructor of the plugin
         */
        ~AdaptiveOpenLocalPlannerROS(){};

        /**
         * @brief Initializes the teb plugin
         * @param name The name of the instance
         * @param tf Pointer to a tf buffer
         * @param costmap_ros Cost map representing occupied and free space
         */
        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

        /**
         * @brief Set the plan that the teb local planner is following
         * @param orig_global_plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        /**
         * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid trajectory was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        /**
         * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
         * @remark Extended version for MBF API
         * @param pose the current pose of the robot.
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
         * @param message Optional more detailed outcome as a string
         * @return

         */
        bool computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                     geometry_msgs::TwistStamped &cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved
         *
         * The actual check is performed in computeVelocityCommands().
         * Only the status flag is checked here.
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        // Functions for subscribing
        void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

        // Local Planning functions
        void extractGlobalPathSection(std::vector<Waypoint> &extracted_path);
        void generateRollOuts(const std::vector<Waypoint> &path, std::vector<std::vector<Waypoint>> &roll_outs);
        void doOneStepStatic(const std::vector<std::vector<Waypoint>> &roll_outs, const std::vector<Waypoint> &extracted_path, std::vector<PathCost> &trajectory_costs, std::vector<Waypoint> &best_path);
        /**
         * @brief velocity is just from waypoint, calculate steering angle rate is needed here
         *
         * @param best_path
         * @param velocity
         * @param steering_angle_rate
         */
        std::vector<Waypoint> calculateVelocityAndSteeringAngleRate(const std::vector<Waypoint> &best_path, float &velocity, float &steering_angle_rate);

        void plot(const std::vector<Waypoint> &waypoint_vec);
        /**
         * @brief Get the Linear Velocity Vec object
         *
         * @param waypoint_vec
         * @return std::vector<std::pair<float,float>> first is distance, second is velocity
         */
        std::vector<std::pair<float, float>> getLinearVelocityVec(const std::vector<Waypoint> &waypoint_vec);

        std::vector<std::pair<float, float>> getJerk(const std::vector<Waypoint> &waypoint_vec);

        bool goalCheck(const std::vector<Eigen::Vector4d> &trajectory);

    protected:
        /**
         * @brief Update internal obstacle vector based on occupied costmap cells
         * @remarks All occupied cells will be added as point obstacles.
         * @remarks All previous obstacles are cleared.
         * @sa updateObstacleContainerWithCostmapConverter
         * @todo Include temporal coherence among obstacle msgs (id vector)
         * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
         */
        void updateObstacleContainerWithCostmap();
        /**
         * @brief convert obstacle in teb into circle and box obstacles
         *
         */
        void convertObstacle();

        void goalCheck();

        /**
         * @brief Prune global plan such that already passed poses are cut off
         *
         * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
         * If no valid transformation can be found, the method returns \c false.
         * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
         * If no pose within the specified treshold \c dist_behind_robot can be found,
         * nothing will be pruned and the method returns \c false.
         * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
         * @param tf A reference to a tf buffer
         * @param global_pose The global pose of the robot
         * @param[in,out] global_plan The plan to be transformed
         * @param dist_behind_robot Distance behind the robot that should be kept [meters]
         * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
         */
        bool pruneGlobalPlan(const tf2_ros::Buffer &tf, const geometry_msgs::PoseStamped &global_pose,
                             std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot = 1);

        /**
         * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
         *
         * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
         * such that the index of the current goal pose is returned as well as
         * the transformation between the global plan and the planning frame.
         * @param tf A reference to a tf buffer
         * @param global_plan The plan to be transformed
         * @param global_pose The global pose of the robot
         * @param costmap A reference to the costmap being used so the window size for transforming can be computed
         * @param global_frame The frame to transform the plan to
         * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also bounded by the local costmap size!]
         * @param[out] transformed_plan Populated with the transformed plan
         * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
         * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
         * @return \c true if the global plan is transformed, \c false otherwise
         */
        bool transformGlobalPlan(const tf2_ros::Buffer &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                 const geometry_msgs::PoseStamped &global_pose, const costmap_2d::Costmap2D &costmap,
                                 const std::string &global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped> &transformed_plan,
                                 int *current_goal_idx = NULL, geometry_msgs::TransformStamped *tf_plan_to_global = NULL) const;

        /**
         * @brief since speed info is not in global planner, so after extracting path from global path, calculate speed is needed
         *
         * @param best_path
         */
        void calculateLinearVelocity(std::vector<Waypoint> &best_path);

        double calculateLinearVelocity(const Waypoint &current_point, const Waypoint &next_point);

        double calculateAngleVelocity(const Waypoint &current_point, const Waypoint &next_point);

        std::vector<Eigen::Vector4d> convertTrajectory(const std::vector<Waypoint> &waypoint_vec);

        bool publishAckermanncmd(const Eigen::Vector2d &control_vec);

        bool publishAckermanncmdstate(const Eigen::Vector4d &predicted_state);

        std::vector<Eigen::Vector4d> fakeTrajectory(const Eigen::Vector4d &current_state);

    private:
        // Definition of member variables

        // external objects (store weak pointers)
        costmap_2d::Costmap2DROS *costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
        costmap_2d::Costmap2D *costmap_;        //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
        tf2_ros::Buffer *tf_;                   //!< pointer to tf buffer

        std::vector<CircleObstacle> circle_obstacles;
        std::vector<BoxObstacle> box_obstacles;

        // internal objects (memory management owned)

        ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization

        std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

        bool goal_reached_; //!< store whether the goal is reached or not

        std::string global_frame_;     //!< The frame in which the controller will run
        std::string robot_base_frame_; //!< Used as the base frame id of the robot

        // flags
        bool initialized_; //!< Keeps track about the correct initialization of this class

        // Subscribers and Publishers
        ros::Subscriber odom_sub;

        ros::Publisher global_plan_rviz_pub;

        ros::Publisher extracted_path_rviz_pub;
        ros::Publisher current_pose_rviz_pub;
        ros::Publisher roll_outs_rviz_pub;
        ros::Publisher weighted_trajectories_rviz_pub;
        ros::Publisher safety_box_rviz_pub;
        ros::Publisher car_footprint_rviz_pub;
        ros::Publisher box_obstacle_rviz_pub;

        ros::Publisher ackermann_cmd_mux_pub_;

        PoseSE2 robot_pose_; //!< Store current robot pose
        // Global Variables
        bool global_plan_received_;
        bool vehicle_state_received_;

        VehicleState current_state_in_map_frame_;

        double prev_cost_;

        ParameterManager params_;
        std::shared_ptr<PathEvaluator> path_evaluator_ptr_;

        std::shared_ptr<VelocityPlanner> velocity_planner_ptr_;

        MPC mpc_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}; // end namespace adaptive_open_local_planner
