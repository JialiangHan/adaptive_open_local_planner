#include <Eigen/Core>
#include "glog/logging.h"
#include "gflags/gflags.h"

namespace adaptive_open_local_planner
{
    class PSO
    {
    public:
        PSO(){};
        PSO(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate);

        /**
         * @brief main loop
         *
         * @return std::vector<float> list of vehicle speed
         */
        std::vector<float> evaluate();

        struct Particle
        {
            // this is position, list of velocity for local path
            std::vector<float> position_vec;
            // this is velocity which update position
            std::vector<float> velocity_vec;
            float cost;
            Particle personal_best;
        };

        typedef std::vector<Particle> Swarm;

    private:
        bool initialize(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &weighting, const float &personal_learning_rate_, const float &global_learning_rate_);

        void setConstraint(const std::vector<std::pair<float, float>> &linear_velocity_boundary);

        void initializeSwarm();

        float evaluateFitnessFunction(const Particle &particle);

        void updateParticle(Particle &particle);

        void updateSwarm();

        void updateGlobalBest();

        float randomFloatNumber(const float &lower_limit, const float &upper_limit);

        std::vector<float> convertGlobalBestToVelocityVec();

    private:
        std::vector<std::vector<Waypoint>> divided_path_;

        float weighting_;

        float personal_learning_rate_;

        float global_learning_rate_;

        std::vector<std::pair<float, float>> linear_velocity_boundary_;

        std::vector<Particle> particle_swarm_;

        Particle global_best_;

        Particle prev_global_best_;
    }
}