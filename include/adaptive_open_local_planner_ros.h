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

#include "glog/logging.h"
#include "gflags/gflags.h"

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
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

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

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
        AdaptiveOpenLocalPlannerROS(){};
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

        void calculateVelocity(const std::vector<Waypoint> &best_path, float &velocity, float &steering_angle_rate);

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

        ros::NodeHandle nh;

        // Hyperparameters
        double planning_frequency_;

        // Parameters
        double MAX_SPEED_;               // max speed that planner should not exceed
        double MAX_LOCAL_PLAN_DISTANCE_; // length of local trajectory roll outs
        double PATH_DENSITY_;            // distance between waypoints of local trajectory
        int ROLL_OUTS_NUMBER_;           // number of roll outs not including the center tracjectory (this number should be even)
        double SAMPLING_TIP_MARGIN_;     // length of car tip margin
        double SAMPLING_OUT_MARGIN_;     // length of roll in margin (??)
        double ROLL_OUT_DENSITY_;        // distance between adjacent trajectories
        double ROLL_IN_SPEED_FACTOR_;
        double ROLL_IN_MARGIN_;
        double LANE_CHANGE_SPEED_FACTOR_;
        double HORIZON_DISTANCE_;

        double HORIZONTAL_SAFETY_DISTANCE_;
        double VERTICAL_SAFETY_DISTANCE_;
        double MAX_STEER_ANGLE_;
        double MIN_SPEED_;
        double LATERAL_SKIP_DISTANCE_;

        double MIN_FOLLOWING_DISTANCE_; // distance threshold for exiting following behaviour
        double MAX_FOLLOWING_DISTANCE_; // distance threshold for entering following behaviour
        double MIN_DISTANCE_TO_AVOID;   // distance threshold for obstacle avoidance behaviour

        double VEHICLE_WIDTH_;
        double VEHICLE_LENGTH_;
        double WHEELBASE_LENGTH_;
        double TURNING_RADIUS_;
        double SAFETY_RADIUS_;

        // Smoothing Weights
        double SMOOTH_DATA_WEIGHT_;
        double SMOOTH_WEIGHT_;
        double SMOOTH_TOLERANCE_;

        double PRIORITY_WEIGHT_;
        double TRANSITION_WEIGHT_;
        double LAT_WEIGHT_;
        double LONG_WEIGHT_;
        double COLLISION_WEIGHT_;
        double CURVATURE_WEIGHT_;

        // Subscribers and Publishers
        ros::Subscriber odom_sub;

        ros::Publisher extracted_path_rviz_pub;
        ros::Publisher current_pose_rviz_pub;
        ros::Publisher roll_outs_rviz_pub;
        ros::Publisher weighted_trajectories_rviz_pub;
        ros::Publisher safety_box_rviz_pub;
        ros::Publisher car_footprint_rviz_pub;
        ros::Publisher box_obstacle_rviz_pub;

        // TF
        tf2_ros::Buffer tf_buffer;

        // Global Variables
        bool global_path_received;
        bool b_vehicle_state;
        bool b_obstacles;

        VehicleState current_state_in_map_frame_;
        VehicleState current_state_in_global_frame_;
        std::vector<Waypoint> global_path;

        int prev_closest_index;
        double prev_cost;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}; // end namespace adaptive_open_local_planner
