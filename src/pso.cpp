#include "pso.h"

namespace adaptive_open_local_planner
{

    PSO::PSO(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle)
    {
        initialize(divided_path, linear_velocity_boundary, weighting, personal_learning_rate, global_learning_rate, cost_difference_boundary, max_interation, number_of_particle);
    }

    bool PSO::initialize(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle)
    {
        // DLOG(INFO) << "size of divided_path is " << divided_path.size();
        divided_path_ = divided_path;

        weighting_ = weighting;

        personal_learning_rate_ = personal_learning_rate;

        global_learning_rate_ = global_learning_rate;

        cost_difference_boundary_ = cost_difference_boundary;

        max_interation_ = max_interation;

        number_of_particle_ = number_of_particle;

        setConstraint(linear_velocity_boundary);
        initializeSwarm();
        DLOG(INFO) << "PSO initialize success!";
        return true;
    }

    std::vector<float> PSO::evaluate()
    {
        std::vector<float> linear_velocity_vec;

        for (size_t iter = 0; iter < max_interation_; iter++)
        {
            updateSwarm();
            updateGlobalBest();
            // end condition
            if (std::abs(global_best_.cost - prev_global_best_.cost) <= cost_difference_boundary_)
            {
                break;
            }
        }
        linear_velocity_vec = convertGlobalBestToVelocityVec();
        return linear_velocity_vec;
    }

    void PSO::setConstraint(const std::vector<std::pair<float, float>> &linear_velocity_boundary)
    {
        // DLOG(INFO) << "size of linear_velocity_boundary is " << linear_velocity_boundary.size();
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
            particle.personal_best.position_vec = particle.position_vec;
            particle.personal_best.cost = particle.cost;
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

        for (size_t i = 0; i < number_of_particle_; i++)
        {
            Particle particle;
            // DLOG(INFO) << "size of linear_velocity_boundary_ is " << linear_velocity_boundary_.size();
            for (size_t i = 0; i < linear_velocity_boundary_.size(); i++)
            {
                // DLOG(INFO) << "index is " << i;
                // initialize position which is vehicle speed
                initial_position = randomFloatNumber(linear_velocity_boundary_[i].first, linear_velocity_boundary_[i].second);
                particle.position_vec.emplace_back(initial_position);
                particle.velocity_vec.emplace_back(0);
            }
            particle.cost = evaluateFitnessFunction(particle);
            particle.personal_best.position_vec = particle.position_vec;

            particle.personal_best.cost = particle.cost;
            particle_swarm_.emplace_back(particle);
        }

        updateGlobalBest();
    }

    float PSO::randomFloatNumber(const float &lower_limit, const float &upper_limit)
    {
        if (std::abs(lower_limit - upper_limit) < 1e-2)
        {
            return lower_limit;
        }

        float r1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        return r1 * (upper_limit - lower_limit);
    }

    float PSO::evaluateFitnessFunction(const Particle &particle)
    {
        // DLOG(INFO) << "in evaluateFitnessFunction";
        float distance, cost = 0;
        // DLOG(INFO) << "size of particle.position_vec is " << particle.position_vec.size();
        for (size_t i = 0; i < particle.position_vec.size() - 1; i++)
        {
            // DLOG(INFO) << "current index is " << i;
            // DLOG(INFO) << "size of divided_path_ is " << divided_path_.size();
            // set distance
            DLOG_IF(FATAL, particle.position_vec.size() != (divided_path_.size() + 1)) << "velocity vec size is not equal to (divided path size +1)";
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

    std::vector<float> PSO::convertGlobalBestToVelocityVec()
    {
        return global_best_.position_vec;
    }

}
