#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include "planner_helpers.h"
#include "struct_defs.h"
#include "matrix.h"
#include "polygon.h"
#include "visualization_helpers.h"

#include <math.h>
#include <limits>
#include <vector>
// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include "adaptive_open_local_planner/Waypoint.h"
#include "adaptive_open_local_planner/WaypointArray.h"
// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace adaptive_open_local_planner
{

    /**
     * @class AdaptiveOpenLocalPlannerROS
     * @brief Implements both nav_core::BaseLocalPlanner and mbf_costmap_core::CostmapController abstract
     * interfaces, so the adaptive_open_local_planner plugin can be used both in move_base and move_base_flex (MBF).
     * @todo Escape behavior, more efficient obstacle handling
     */
    class AdaptiveOpenLocalPlannerROS : public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController
    {

    public:
        /**
         * @brief Default constructor of the teb plugin
         */
        AdaptiveOpenLocalPlannerROS();

        /**
         * @brief  Destructor of the plugin
         */
        ~AdaptiveOpenLocalPlannerROS();

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
         * @param velocity the current velocity of the robot.
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
         * @param message Optional more detailed outcome as a string
         * @return Result code as described on ExePath action result:
         *         SUCCESS         = 0
         *         1..9 are reserved as plugin specific non-error results
         *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
         *         CANCELED        = 101
         *         NO_VALID_CMD    = 102
         *         PAT_EXCEEDED    = 103
         *         COLLISION       = 104
         *         OSCILLATION     = 105
         *         ROBOT_STUCK     = 106
         *         MISSED_GOAL     = 107
         *         MISSED_PATH     = 108
         *         BLOCKED_PATH    = 109
         *         INVALID_PATH    = 110
         *         TF_ERROR        = 111
         *         NOT_INITIALIZED = 112
         *         INVALID_PLUGIN  = 113
         *         INTERNAL_ERROR  = 114
         *         121..149 are reserved as plugin specific errors
         */
        uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped &pose, const geometry_msgs::TwistStamped &velocity,
                                         geometry_msgs::TwistStamped &cmd_vel, std::string &message);

        /**
         * @brief  Check if the goal pose has been achieved
         *
         * The actual check is performed in computeVelocityCommands().
         * Only the status flag is checked here.
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief Dummy version to satisfy MBF API
         */
        bool isGoalReached(double xy_tolerance, double yaw_tolerance) { return isGoalReached(); };

        // Main Function
        void mainTimerCallback(const ros::TimerEvent &timer_event);

        // Functions for subscribing
        void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
        void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr &obstacle_msg);
        void globalPathCallback(const junior_local_planner::WaypointArray::ConstPtr &global_path_msg);

        // Functions for publishing results
        void publishCmdVel();

        // Local Planning functions
        void extractGlobalPathSection(std::vector<Waypoint> &extracted_path);
        void generateRollOuts(const std::vector<Waypoint> &path, std::vector<std::vector<Waypoint>> &roll_outs);
        PathCost doOneStepStatic(const std::vector<std::vector<Waypoint>> &roll_outs, const std::vector<Waypoint> &extracted_path,
                                 std::vector<PathCost> &trajectory_costs);

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
         * @brief Update internal obstacle vector based on polygons provided by a costmap_converter plugin
         * @remarks Requires a loaded costmap_converter plugin.
         * @remarks All previous obstacles are cleared.
         * @sa updateObstacleContainerWithCostmap
         */
        void updateObstacleContainerWithCostmapConverter();

    private:
        // Definition of member variables

        // external objects (store weak pointers)
        costmap_2d::Costmap2DROS *costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
        costmap_2d::Costmap2D *costmap_;        //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
        tf2_ros::Buffer *tf_;                   //!< pointer to tf buffer

        std::vector<CircleObstacle> circle_obstacles;
        std::vector<BoxObstacle> box_obstacles;

        // internal objects (memory management owned)
        PlannerInterfacePtr planner_;       //!< Instance of the underlying optimal planner class
        ObstContainer obstacles_;           //!< Obstacle vector that should be considered during local trajectory optimization
        ViaPointContainer via_points_;      //!< Container of via-points that should be considered during local trajectory optimization
        TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
        boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
        TebConfig cfg_; //!< Config class that stores and manages all related parameters

        std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

        base_local_planner::OdometryHelperRos odom_helper_; //!< Provides an interface to receive the current velocity from the robot

        pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
        boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;             //!< Store the current costmap_converter

        boost::shared_ptr<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>> dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
        ros::Subscriber custom_obst_sub_;                                                                //!< Subscriber for custom obstacles received via a ObstacleMsg.
        boost::mutex custom_obst_mutex_;                                                                 //!< Mutex that locks the obstacle array (multi-threaded)
        costmap_converter::ObstacleArrayMsg custom_obstacle_msg_;                                        //!< Copy of the most recent obstacle message

        ros::Subscriber via_points_sub_; //!< Subscriber for custom via-points received via a Path msg.
        bool custom_via_points_active_;  //!< Keep track whether valid via-points have been received from via_points_sub_
        boost::mutex via_point_mutex_;   //!< Mutex that locks the via_points container (multi-threaded)

        PoseSE2 robot_pose_;                  //!< Store current robot pose
        PoseSE2 robot_goal_;                  //!< Store current robot goal
        geometry_msgs::Twist robot_vel_;      //!< Store current robot translational and angular velocity (vx, vy, omega)
        bool goal_reached_;                   //!< store whether the goal is reached or not
        ros::Time time_last_infeasible_plan_; //!< Store at which time stamp the last infeasible plan was detected
        int no_infeasible_plans_;             //!< Store how many times in a row the planner failed to find a feasible plan.
        ros::Time time_last_oscillation_;     //!< Store at which time stamp the last oscillation was detected
        RotType last_preferred_rotdir_;       //!< Store recent preferred turning direction
        geometry_msgs::Twist last_cmd_;       //!< Store the last control command generated in computeVelocityCommands()

        std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot
        double robot_inscribed_radius_;                    //!< The radius of the inscribed circle of the robot (collision possible)
        double robot_circumscribed_radius;                 //!< The radius of the circumscribed circle of the robot

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
        ros::Subscriber obstacles_sub;
        ros::Subscriber global_path_sub;

        ros::Publisher global_path_rviz_pub;
        ros::Publisher extracted_path_rviz_pub;
        ros::Publisher current_pose_rviz_pub;
        ros::Publisher roll_outs_rviz_pub;
        ros::Publisher weighted_trajectories_rviz_pub;
        ros::Publisher safety_box_rviz_pub;
        ros::Publisher car_footprint_rviz_pub;
        ros::Publisher box_obstacle_rviz_pub;
        ros::Publisher cmd_vel_pub;

        // Timer
        ros::Timer timer;

        // TF
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // Global Variables
        bool b_global_path;
        bool b_vehicle_state;
        bool b_obstacles;

        VehicleState current_state;
        std::vector<Waypoint> global_path;

        int prev_closest_index;
        double prev_cost;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}; // end namespace adaptive_open_local_planner
