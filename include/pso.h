#include <Eigen/Core>
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "struct_defs.h"
#include "planner_helpers.h"
#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include "matplotlibcpp.h"
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
         * @return std::vector<float> list of vehicle speed, first item is velocity vec, second one is angular velocity vec
         */
        std::vector<Waypoint> evaluate();
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

        std::vector<float> findJerk(const Particle &particle);

        std::vector<std::pair<float, float>> findJerk();

        std::vector<std::pair<float, float>> findJerkPair(const Particle &particle);

    private:
        float sumJerk(const Particle &particle);

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

        float handleJerkConstraintsForFitnessFunction(const Particle &particle);

        float handleAccelerationConstraintsForFitnessFunction(const Particle &particle);

        void updateParticle(Particle &particle);

        void updateSwarm();

        void updateGlobalBest();

        float randomFloatNumber(const float &lower_limit, const float &upper_limit);

        std::vector<float> convertGlobalBestToVelocityVec();

        std::vector<float> findAcceleration(const Particle &particle);

        void updateParticleToVelocityBoundary(Particle &particle);

        void publishJerk();

        void publishCost();

        /**
         * @brief coarse velocity vec got from PSO should be converted to a fine velocity vec. V(t)=A*t^3+B*t^2+C*t+D, A=-2*(v1-v0)/t1^3;B=3(v1-v0)/t1^2;C=0;D=v0; t1=2s/(v0+v1)
         *
         * @param coarse_velocity_vec
         * @return std::vector<float>
         */
        void findFineVelocityVec(const std::vector<float> &coarse_velocity_vec);

        void findAngularVelocity();

        std::vector<Waypoint> convertDividedPathToFullPath();

        void setVelocityForWayPoint(float start_velocity, float end_velocity, float time_limit, float A, float B, float C, float D, std::vector<Waypoint> &path);

        float getVelocity(float time, float A, float B, float C, float D);
        /**
         * @brief distance=(A*t^4)4+(B*t^3)/3+D*t
         *
         * @param A
         * @param B
         * @param C
         * @param D
         * @param start_velocity
         * @param distance
         * @return float
         */
        float solveTime(float A, float B, float C, float D, float distance, float time_limit);
        /**
         * @brief
         *
         * @param start_velocity
         * @param end_velocity
         * @param distance
         * @return std::vector<float> A,B,C,D
         */
        std::vector<float> findParameter(float start_velocity, float end_velocity, float distance);

        void plotCost();

    private:
        std::vector<std::vector<Waypoint>> divided_path_;
        // time limit is also need
        std::vector<std::vector<float>> path_parameter_vec_;

        std::vector<float> cost_vec_;

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

        ros::Publisher cost_pub_;
    };
};