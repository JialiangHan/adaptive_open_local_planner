#include "pso.h"

namespace adaptive_open_local_planner
{

    PSO::PSO(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate)
    {
        initialize(divided_path, linear_velocity_boundary, weighting, personal_learning_rate, global_learning_rate);
    }

    bool PSO::initialize(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate)
    {
        divided_path_ = divided_path;

        weighting_ = weighting;

        personal_learning_rate_ = personal_learning_rate;

        global_learning_rate_ = global_learning_rate;

        setConstraint(linear_velocity_boundary);
        initializeSwarm();
        DLOG(INFO) << "PSO initialize success!";
        return true;
    }

    std::vector<float> evaluate()
    {
        std::vector<float> linear_velocity_vec;
        float cost_difference_boundary = ? ? ? ;
        int max_interation = ? ? ;
        for (size_t iter = 0; iter < max_iteration; iter++)
        {
            updateSwarm();
            updateGlobalBest();
            // end condition
            if (std::abs(global_best_.cost - prev_global_best_.cost) <= cost_difference_boundary)
            {
                break;
            }
        }
        linear_velocity_vec = convertGlobalBestToVelocityVec();
        return linear_velocity_vec;
    }

    void PSO::setConstraint(const std::vector<std::pair<float, float>> &linear_velocity_boundary)
    {
        linear_velocity_boundary_ = linear_velocity_boundary;
    }

    void PSO::updateParticle(Particle &particle)
    {
        float r1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX), r2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        for (uint index = 0; index < particle.velocity_vec.size(); index++)
        {
            // first update velocity
            particle.velocity_vec[index] = weighting_ * particle.velocity_vec[index] + personal_learning_rate_ * r1 * (particle.personal_best.position_vec[index] - particle.position_vec[index]) + global_learning_rate_ * r2 * (global_best_.position_vec[index] - particle.position_vec[index]);
            // second update location
            particle.position_vec[index] = particle.position_vec[index] + particle.velocity_vec[index];
        }
        // 3. update cost
        particle.cost = evaluateFitnessFunction(particle);
        // 4. update personal best
        if (particle.cost < particle.personal_best.cost)
        {
            particle.personal_best = particle;
        }
    }

    void PSO::updateSwarm()
    {
        if (particle_swarm_.size() < 1)
        {
            DLOG(WARNING) << "size of particle swarm is wrong!!!";
            return;
        }

        for (auto &particle : particle_swarm_)
        {
            updateParticle(particle);
        }
    }

    void PSO::updateGlobalBest()
    {
        // reset global best to first particle
        if (particle_swarm_.size() >= 1)
        {
            global_best_ = particle_swarm_[0];
            prev_global_best_ = particle_swarm_[0];
        }
        else
        {
            DLOG(WARNING) << "size of particle swarm is wrong!!!";
        }

        for (const auto &particle : particle_swarm_)
        {
            if (global_best_.cost > particle.cost)
            {
                prev_global_best_ = global_best_;
                global_best_ = particle;
            }
        }
    }

    void PSO::initializeSwarm()
    {
        // check if constraint is set
        if (linear_velocity_boundary_.size() == 0)
        {
            DLOG(WARNING) << "linear velocity boundary not set!!!";
        }
        float initial_position;
        for (size_t i = 0; i < linear_velocity_boundary_.size(); i++)
        {
            // initialize position which is vehicle speed
            initial_position = randomFloatNumber(linear_velocity_boundary_[i].first, linear_velocity_boundary_[i].second);
            particle_swarm_[i].position_vec.emplace_back(initial_position);
            // initialize velocity to zero
            particle_swarm_[i].velocity_vec.emplace_back(0);
            // initialize cost
            particle_swarm_[i].cost = evaluateFitnessFunction(particle_swarm_[i]);
            // initialize personal best to itself
            particle_swarm_[i].personal_best = particle_swarm_[i];
        }
        updateGlobalBest();
    }

    float PSO::randomFloatNumber(const float &lower_limit, const float &upper_limit)
    {
        float r1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        return r1 * (upper_limit - lower_limit);
    }

    float PSO::evaluateFitnessFunction(const Particle &particle)
    {
        float distance, cost = 0;
        for (size_t i = 0; i < particle.position_vec.size() - 1; i++)
        {
            // set distance
            distance = PlannerHelpers::getDistance(divided_path_[i]);
            // calculate cost
            if ((particle.position_vec[i] + particle.position_vec[i + 1]) != 0)
            {
                cost = cost + 2 * distance / (particle.position_vec[i] + particle.position_vec[i + 1]);
            }
            else
            {
                // set to large value to show speed is zero
                cost = cost + 100000;
            }
        }
        return cost;
    }

    std::vector<float> convertGlobalBestToVelocityVec()
    {
        return global_best_.position_vec;
    }

}
