#include <Eigen/Core>
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "struct_defs.h"
#include "planner_helpers.h"
#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
namespace adaptive_open_local_planner
{
    class PSO
    {
    public:
        PSO(){};
        PSO(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &min_acceleration, const float &max_acceleration, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle);

        /**
         * @brief main loop
         *
         * @return std::vector<float> list of vehicle speed
         */
        std::vector<float> evaluate();
        struct PersonalBest
        {
            // this is position, list of velocity for local path
            std::vector<float> position_vec;
            float cost = INFINITY;
        };

        struct Particle
        {
            // this is position, list of velocity for local path
            std::vector<float> position_vec;
            // this is velocity which update position
            std::vector<float> velocity_vec;
            float cost;
            PersonalBest personal_best;
        };

        typedef std::vector<Particle> Swarm;

    private:
        bool initialize(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &min_acceleration, const float &max_acceleration, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle);

        void setConstraint(const std::vector<std::pair<float, float>> &linear_velocity_boundary);

        void initializeSwarm();

        float evaluateFitnessFunction(const Particle &particle);
        /**
         * @brief evaluate term of constraints in fitness function velocity and acceleration
         *
         * @param particle
         * @return float
         */
        float evaluateFitnessFunctionConstraints(const Particle &particle);

        float handleVelocityConstraintsForFitnessFunction(const Particle &particle);

        float handleAccelerationConstraintsForFitnessFunction(const Particle &particle);

        void updateParticle(Particle &particle);

        void updateSwarm();

        void updateGlobalBest();

        float randomFloatNumber(const float &lower_limit, const float &upper_limit);

        std::vector<float> convertGlobalBestToVelocityVec();

        std::vector<float> findAcceleration(const Particle &particle);

        std::vector<float> findJerk(const Particle &particle);

        void updateParticleToVelocityBoundary(Particle &particle);

        void publishJerk();

    private:
        std::vector<std::vector<Waypoint>> divided_path_;

        float weighting_;

        float personal_learning_rate_;

        float global_learning_rate_;

        float cost_difference_boundary_;

        float max_interation_;

        int number_of_particle_;

        std::vector<std::pair<float, float>> linear_velocity_boundary_;

        float max_acceleration_;

        float min_acceleration_;

        std::vector<Particle> particle_swarm_;

        Particle global_best_;

        Particle prev_global_best_;

        ros::NodeHandle nh_;

        ros::Publisher jerk_pub_;
    };
};