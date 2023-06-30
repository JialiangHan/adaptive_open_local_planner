#include "pso.h"

namespace adaptive_open_local_planner
{

    PSO::PSO(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &min_acceleration, const float &max_acceleration, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle)
    {
        initialize(divided_path, linear_velocity_boundary, min_acceleration, max_acceleration, weighting, personal_learning_rate, global_learning_rate, cost_difference_boundary, max_interation, number_of_particle);
    }

    bool PSO::initialize(const std::vector<std::vector<Waypoint>> &divided_path, const std::vector<std::pair<float, float>> &linear_velocity_boundary, const float &min_acceleration, const float &max_acceleration, const float &weighting, const float &personal_learning_rate, const float &global_learning_rate, const float &cost_difference_boundary, const int &max_interation, const int &number_of_particle)
    {
        // DLOG(INFO) << "size of divided_path is " << divided_path.size();
        divided_path_ = divided_path;
        for (const auto &unit : divided_path_)
        {
            for (const auto &element : unit)
            {
                DLOG_IF(INFO, std::isnan(element.x) || std::isnan(element.y)) << "waypoint is NAN. current point is " << element.x << " " << element.y;
            }
        }

        weighting_ = weighting;

        personal_learning_rate_ = personal_learning_rate;

        global_learning_rate_ = global_learning_rate;

        cost_difference_boundary_ = cost_difference_boundary;

        max_interation_ = max_interation;

        number_of_particle_ = number_of_particle;

        min_acceleration_ = min_acceleration;
        max_acceleration_ = max_acceleration;

        setConstraint(linear_velocity_boundary);
        initializeSwarm();
        // DLOG(INFO) << "PSO initialize success!";
        return true;
    }

    std::vector<Waypoint> PSO::evaluate()
    {
        std::vector<float> linear_velocity_vec;
        std::vector<float> angular_velocity_vec;
        std::vector<std::vector<float>> velocity_vec;
        for (size_t iter = 0; iter < max_interation_; iter++)
        {
            updateSwarm();
            updateGlobalBest();
            // end condition
            if (std::abs(global_best_.cost - prev_global_best_.cost) <= cost_difference_boundary_)
            {
                // DLOG(INFO) << "current global best cost is " << global_best_.cost << " previous global best cost is " << prev_global_best_.cost << " difference is smaller than preset value: " << cost_difference_boundary_ << " current iteration number is " << iter;
                break;
            }
        }
        // DLOG(INFO) << "current global best cost is " << global_best_.cost << " previous global best cost is " << prev_global_best_.cost << " difference is smaller than preset value: " << cost_difference_boundary_;
        linear_velocity_vec = convertGlobalBestToVelocityVec();
        for (int index = 0; index < linear_velocity_vec.size(); index++)
        {
            // DLOG(INFO) << "velocity is " << linear_velocity_vec[index] << " upper velocity boundary is " << linear_velocity_boundary_[index].second << " diff is " << linear_velocity_boundary_[index].second - linear_velocity_vec[index];
        }
        publishJerk();
        findAngularVelocity();
        return convertDividedPathToFullPath();
        // return velocity_vec;
    }

    void PSO::findAngularVelocity()
    {
        // DLOG(INFO) << "in findAngularVelocity:";
        findFineVelocityVec(global_best_.position_vec);

        float angular_velocity = 0;

        // set dt to a constant value
        float angle_diff, delta_time, delta_distance;
        Waypoint next_point, current_point;
        // DLOG(INFO) << "size of fine_velocity_vec is " << fine_velocity_vec.size();

        // for (const auto &element : full_path_vec)
        // {
        //     DLOG_IF(INFO, std::isnan(element.x) || std::isnan(element.y)) << "waypoint is NAN. current point is " << element.x << " " << element.y;
        // }
        // DLOG(INFO) << "size of fine_velocity_vec is " << fine_velocity_vec.size() << " size of full path vec is " << full_path_vec.size();
        // for (int index = 0; index < fine_velocity_vec.size() - 1; index++)
        // {

        // if (index < divided_path_[0].size())
        // {
        //     current_point = divided_path_[0][index];
        //     next_point = divided_path_[0][index + 1];
        //     // DLOG(INFO) << "index smaller than divided_path_[0].size()";
        //     DLOG_IF(INFO, std::isnan(current_point.x) || std::isnan(current_point.y) || std::isnan(next_point.y) || std::isnan(next_point.y)) << "waypoint is NAN. current point is " << current_point.x << " " << current_point.y << "next point is " << next_point.x << " " << next_point.y << " index is " << index;
        //     DLOG_IF(INFO, std::isnan(divided_path_[0][index].x) || std::isnan(divided_path_[0][index].y) || std::isnan(divided_path_[0][index + 1].y) || std::isnan(divided_path_[0][index + 1].y)) << "waypoint is NAN. divided_path_[0][index] is " << divided_path_[0][index].x << " " << divided_path_[0][index].y << "divided_path_[0][index + 1] is " << divided_path_[0][index + 1].x << " " << divided_path_[0][index + 1].y << " index is " << index;
        // }
        // else
        // {
        //     // DLOG(INFO) << "index larger than divided_path_[0].size()";
        //     current_point = divided_path_[1][index - divided_path_[0].size()];
        //     next_point = divided_path_[1][index - divided_path_[0].size() + 1];
        //     DLOG_IF(INFO, std::isnan(current_point.x) || std::isnan(current_point.y) || std::isnan(next_point.y) || std::isnan(next_point.y)) << "waypoint is NAN. current point is " << current_point.x << " " << current_point.y << "next point is " << next_point.x << " " << next_point.y << " index - divided_path_[0].size() is " << index - divided_path_[0].size();
        //     DLOG_IF(INFO, std::isnan(divided_path_[1][index - divided_path_[0].size()].x) || std::isnan(divided_path_[1][index - divided_path_[0].size()].y) || std::isnan(divided_path_[1][index - divided_path_[0].size() + 1].y) || std::isnan(divided_path_[1][index - divided_path_[0].size() + 1].y)) << "waypoint is NAN. divided_path_[1][index - divided_path_[0].size()] is " << divided_path_[1][index - divided_path_[0].size()].x << " " << divided_path_[1][index - divided_path_[0].size()].y << "divided_path_[1][index - divided_path_[0].size() + 1] is " << divided_path_[1][index - divided_path_[0].size() + 1].x << " " << divided_path_[1][index - divided_path_[0].size() + 1].y << " index is " << index - divided_path_[0].size();
        // }
        for (int i = 0; i < divided_path_.size(); i++)
        {
            for (int j = 0; j < divided_path_[i].size() - 1; j++)
            {
                delta_distance = PlannerHelpers::getDistance(divided_path_[i][j], divided_path_[i][j + 1]);
                angle_diff = divided_path_[i][j + 1].heading - divided_path_[i][j].heading;
                delta_time = 2 * delta_distance / (divided_path_[i][j + 1].speed + divided_path_[i][j].speed);

                angular_velocity = angle_diff / delta_time;
                divided_path_[i][j].angular_speed = angular_velocity;
            }
        }

        // }

        // DLOG(INFO) << "out findAngularVelocity";
        // return angular_velocity_vec;
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
            // DLOG(INFO) << "current velocity(PSO) is " << particle.velocity_vec[index] << " current position is " << particle.position_vec[index];
            // first update velocity
            particle.velocity_vec[index] = weighting_ * particle.velocity_vec[index] + personal_learning_rate_ * r1 * (particle.personal_best.position_vec[index] - particle.position_vec[index]) + global_learning_rate_ * r2 * (global_best_.position_vec[index] - particle.position_vec[index]);
            // second update location
            particle.position_vec[index] = particle.position_vec[index] + particle.velocity_vec[index];
            // DLOG(INFO) << "after update: current velocity(PSO) is " << particle.velocity_vec[index] << " current position is " << particle.position_vec[index];
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
    // this function might be changed due to cost function is changed.
    void PSO::updateParticleToVelocityBoundary(Particle &particle)
    {
        // create a new particle, compare their cost, if this cost is lower, than replace it. else do nothing
        Particle new_particle = particle;
        float r1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        for (uint index = 0; index < new_particle.position_vec.size(); index++)
        {
            // DLOG(INFO) << "current velocity(PSO) is " << particle.velocity_vec[index] << " current position is " << particle.position_vec[index];

            new_particle.position_vec[index] = new_particle.position_vec[index] + r1 * (linear_velocity_boundary_[index].second - new_particle.position_vec[index]);
            // DLOG(INFO) << "after update: current velocity(PSO) is " << particle.velocity_vec[index] << " current position is " << particle.position_vec[index];
        }
        // update cost
        new_particle.cost = evaluateFitnessFunction(new_particle);
        if (new_particle.cost < particle.cost)
        {
            particle = new_particle;
            if (particle.cost < particle.personal_best.cost)
            {
                particle.personal_best.position_vec = particle.position_vec;
                particle.personal_best.cost = particle.cost;
            }
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
            updateParticleToVelocityBoundary(particle);
        }
    }
    // checked OK
    void PSO::updateGlobalBest()
    {
        // DLOG(INFO) << "In updateGlobalBest:";
        // reset global best to first particle
        if (particle_swarm_.size() >= 1)
        {
            global_best_ = particle_swarm_[0];
            prev_global_best_ = particle_swarm_[0];
            // DLOG(INFO) << "reset global best and previous global best to first one, cost is " << particle_swarm_[0].cost;
        }
        else
        {
            DLOG(WARNING) << "size of particle swarm is wrong!!!";
        }

        for (const auto &particle : particle_swarm_)
        {
            // DLOG(INFO) << "cost is " << particle.cost;
            if (global_best_.cost > particle.cost)
            {
                // DLOG(INFO) << "set global best cost is " << particle.cost << " previous global best cost is " << global_best_.cost;
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

                // initialize position which is vehicle speed
                initial_position = randomFloatNumber(linear_velocity_boundary_[i].first, linear_velocity_boundary_[i].second);
                // change initial position to max velocity
                // initial_position = std::max(linear_velocity_boundary_[i].first, linear_velocity_boundary_[i].second);
                particle.position_vec.emplace_back(initial_position);
                particle.velocity_vec.emplace_back(0);
                // DLOG(INFO) << "index is " << i << " initial position is " << initial_position << " initial speed is " << 0;
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
        // original cost function is cost= distance/velocity.
        // DLOG(INFO) << "in evaluateFitnessFunction";
        float distance, cost = 0, jerk_portion, delta_velocity, time;
        // DLOG(INFO) << "size of particle.position_vec is " << particle.position_vec.size();
        // DLOG(INFO) << "size of divided_path_ is " << divided_path_.size();
        for (size_t i = 0; i < particle.position_vec.size() - 1; i++)
        {
            // DLOG(INFO) << "current index is " << i;
            // set distance
            DLOG_IF(FATAL, particle.position_vec.size() != (divided_path_.size() + 1)) << "velocity vec size is not equal to (divided path size +1)";
            distance = PlannerHelpers::getDistance(divided_path_[i]);
            delta_velocity = (particle.position_vec[i + 1] + particle.position_vec[i]);

            // calculate cost
            if ((particle.position_vec[i] + particle.position_vec[i + 1]) != 0)
            {
                time = 2 * distance / (particle.position_vec[i] + particle.position_vec[i + 1]);
                cost = cost + time;
            }
            else
            {
                // set to large value to show speed is zero
                cost = cost + 100000;
            }
        }
        // DLOG(INFO) << "velocity is " << cost;
        cost = cost + evaluateFitnessFunctionConstraints(particle);
        // DLOG(INFO) << "constraints cost is " << evaluateFitnessFunctionConstraints(particle);
        // DLOG(INFO) << "cost is " << cost;
        bool add_jerk = false;
        if (add_jerk)
        {
            jerk_portion = std::abs(PlannerHelpers::sumVector(findJerk(particle)));
            // DLOG(INFO) << "jerk portion cost is " << jerk_portion;
            cost = cost + jerk_portion;
        }
        // DLOG(INFO) << "cost is " << cost;
        return cost;
    }

    std::vector<float> PSO::convertGlobalBestToVelocityVec()
    {
        return global_best_.position_vec;
    }

    float PSO::evaluateFitnessFunctionConstraints(const Particle &particle)
    {
        float final_cost = 0, velocity_portion, acceleration_portion;
        // set a large constant
        int weight = 1000000;
        // 1. handle velocity constraints
        velocity_portion = handleVelocityConstraintsForFitnessFunction(particle);
        // 2. handle acceleration constraints
        acceleration_portion = handleAccelerationConstraintsForFitnessFunction(particle);
        // DLOG(INFO) << "velocity_portion is " << velocity_portion << " acceleration_portion is " << acceleration_portion;
        final_cost = weight * velocity_portion + weight * acceleration_portion;
        return final_cost;
    }

    float PSO::handleAccelerationConstraintsForFitnessFunction(const Particle &particle)
    {
        float total_cost = 0;
        std::vector<float> acceleration_vec = findAcceleration(particle);
        for (const auto &acceleration : acceleration_vec)
        {
            // DLOG(INFO) << "total cost is " << total_cost;
            // DLOG(INFO) << "acceleration is " << acceleration << " min_acceleration_ is " << min_acceleration_ << " max_acceleration_ is " << max_acceleration_;
            // lower limit
            total_cost = total_cost + std::abs(std::min(acceleration - min_acceleration_, (float)0));
            // DLOG(INFO) << "total cost is " << total_cost;
            // upper limit
            total_cost = total_cost + std::max(acceleration - max_acceleration_, (float)0);
            // DLOG(INFO) << "total cost is " << total_cost;
        }

        return total_cost;
    }

    std::vector<float> PSO::findAcceleration(const Particle &particle)
    {
        std::vector<float> acceleration_vec;
        // acceleration=velocity / time, velocity should use average velocity, time = distance / average velocity
        float distance, avg_velocity, time, acceleration;
        for (size_t i = 0; i < divided_path_.size(); i++)
        {
            // 1. find distance
            distance = PlannerHelpers::getDistance(divided_path_[i]);
            // 2. find average velocity
            avg_velocity = (particle.position_vec[i] + particle.position_vec[i + 1]) / 2;
            // 3. get time
            if (avg_velocity != 0)
            {
                time = distance / avg_velocity;
            }
            else
            {
                time = 100000000;
                DLOG(WARNING) << "average velocity is zero!!!";
            }
            if (time != 0)
            {
                acceleration = (particle.position_vec[i + 1] - particle.position_vec[i]) / time;
            }
            else
            {
                acceleration = 100000000;
                DLOG(WARNING) << "time is zero!!!";
            }
            acceleration_vec.emplace_back(acceleration);
        }
        return acceleration_vec;
    }

    std::vector<float> PSO::findJerk(const Particle &particle)
    {
        // DLOG(INFO) << "in findJerk";
        std::vector<float> jerk_vec;
        // acceleration=velocity / time, velocity should use average velocity, time = distance / average velocity
        float distance, jerk, time_limit, start_velocity, end_velocity, A, B, C, D;
        for (size_t i = 0; i < divided_path_.size(); i++)
        {
            // set velocity
            start_velocity = particle.position_vec[i];
            end_velocity = particle.position_vec[i + 1];
            // set distance
            distance = PlannerHelpers::getDistance(divided_path_[i]);
            // set time: t1=2s/(v0+v1)
            if ((start_velocity + end_velocity) != 0)
            {
                time_limit = 2 * distance / (start_velocity + end_velocity);
            }
            else
            {
                DLOG(FATAL) << "start velocity + end velocity is zero!!!";
            }

            // A=-2*(v1-v0)/t1^3;B=3(v1-v0)/t1^2;C=0;D=v0
            A = -2 * (end_velocity - start_velocity) / std::pow(time_limit, 3);
            B = 3 * (end_velocity - start_velocity) / std::pow(time_limit, 2);
            C = 0;
            D = start_velocity;
            // DLOG(INFO) << "time limit is " << time_limit;
            for (float time = 0; time < time_limit; time = time + 0.01 * time_limit)
            {
                jerk = 6 * A * time + 2 * B;

                jerk_vec.emplace_back(jerk);
                // DLOG(INFO) << "time is " << time;
            }
        }
        // DLOG(INFO) << "out findJerk";
        return jerk_vec;
    }

    float PSO::handleVelocityConstraintsForFitnessFunction(const Particle &particle)
    {
        float total_cost = 0;

        for (int i = 0; i < particle.position_vec.size(); i++)
        {
            // lower limit
            total_cost = total_cost + std::min(particle.position_vec[i] - linear_velocity_boundary_[i].first, (float)0);
            // upper limit
            total_cost = total_cost + std::max(particle.position_vec[i] - linear_velocity_boundary_[i].second, (float)0);
        }

        return total_cost;
    }

    void PSO::publishJerk()
    {
        std::vector<float> jerk_vec = findJerk(global_best_);
        jerk_pub_ = nh_.advertise<std_msgs::Float32>("jerk", 1, true);
        std_msgs::Float32 jerk;
        if (jerk_vec.size() != 0)
        {
            jerk.data = jerk_vec[0];
            jerk_pub_.publish(jerk);
        }
        else
        {
            DLOG(WARNING) << "size of jerk vec is zero!!!";
        }
    }

    void PSO::findFineVelocityVec(const std::vector<float> &coarse_velocity_vec)
    {
        // DLOG(INFO) << "in findFineVelocityVec";
        std::vector<float> fine_velocity_vec;
        float distance, time_limit, start_velocity, end_velocity, A, B, C, D;
        // DLOG(INFO) << "coarse_velocity_vec size is " << coarse_velocity_vec.size();
        for (size_t i = 0; i < coarse_velocity_vec.size() - 1; i++)
        {
            // set velocity
            start_velocity = coarse_velocity_vec[i];
            end_velocity = coarse_velocity_vec[i + 1];
            // set distance
            distance = PlannerHelpers::getDistance(divided_path_[i]);
            // set time: t1=2s/(v0+v1)
            if ((start_velocity + end_velocity) != 0)
            {
                time_limit = 2 * distance / (start_velocity + end_velocity);
            }
            else
            {
                DLOG(FATAL) << "start velocity + end velocity is zero!!!";
            }

            // A=-2*(v1-v0)/t1^3;B=3(v1-v0)/t1^2;C=0;D=v0
            A = -2 * (end_velocity - start_velocity) / std::pow(time_limit, 3);
            B = 3 * (end_velocity - start_velocity) / std::pow(time_limit, 2);
            C = 0;
            D = start_velocity;

            setVelocityForWayPoint(start_velocity, end_velocity, time_limit, A, B, C, D, divided_path_[i]);
            // DLOG(INFO) << "time limit is " << time_limit;
            // for (float time = 0; time < time_limit; time = time + 0.01 * time_limit)
            // {
            //     float temp_velocity = A * std::pow(time, 3) + B * std::pow(time, 2) + C * time + D;

            //     fine_velocity_vec.emplace_back(temp_velocity);
            //     // DLOG(INFO) << "time is " << time;
            // }
        }
        // DLOG(INFO) << "out findFineVelocityVec";
        // return fine_velocity_vec;
    }

    std::vector<Waypoint> PSO::convertDividedPathToFullPath()
    {
        std::vector<Waypoint> full_path_vec;
        for (const auto &element_vec : divided_path_)
        {
            for (const auto &element : element_vec)
            {
                full_path_vec.emplace_back(element);
            }
        }
        return full_path_vec;
    }
    // checked, it`s ok
    void PSO::setVelocityForWayPoint(float start_velocity, float end_velocity, float time_limit, float A, float B, float C, float D, std::vector<Waypoint> &path)
    {
        // DLOG(INFO) << "in setVelocityForWayPoint";
        if (path.size() < 2)
        {
            DLOG(WARNING) << "path size is too small, size is " << path.size();
            return;
        }

        path[0].speed = start_velocity;
        path.back().speed = end_velocity;
        float time, velocity, distance;
        // DLOG(INFO) << "path size is " << path.size() << " start velocity is " << path[0].speed << " end velocity is " << path.back().speed;
        for (size_t i = 1; i < path.size() - 1; i++)
        {
            // DLOG(INFO) << "index is " << i << " path size is " << path.size();
            DLOG_IF(INFO, i == (path.size() - 1)) << "last node for current path, current speed is " << path[i].speed;
            if (start_velocity == end_velocity)
            {
                path[i].speed = start_velocity;
                // DLOG(INFO) << "start velocity is equal to end velocity, set velocity to " << start_velocity << " for index " << i;
                continue;
            }

            distance = PlannerHelpers::getDistance(path[0], path[i]);
            time = solveTime(A, B, C, D, distance, time_limit);
            velocity = getVelocity(time, A, B, C, D);
            path[i].speed = velocity;
            // DLOG(INFO) << "set velocity to " << velocity;
        }
        // DLOG(INFO) << "out setVelocityForWayPoint";
    }

    float PSO::solveTime(float A, float B, float C, float D, float distance, float time_limit)
    {
        // DLOG(INFO) << "in solveTime";
        float time, temp, time_lower_limit = 0, time_upper_limit = time_limit;

        while (1)
        {
            time = 0.5 * (time_upper_limit - time_lower_limit) + time_lower_limit;
            temp = 0.25 * A * std::pow(time, 4) + B * std::pow(time, 3) / 3 + D * time;
            if (std::abs(temp - distance) < 1e-3)
            {
                // DLOG(INFO) << "time found: " << time << " distance is " << distance;
                break;
            }

            if (temp > distance)
            {
                time_upper_limit = time;
            }
            else
            {
                time_lower_limit = time;
            }
        }
        // DLOG(INFO) << "out solveTime";
        return time;
    }

    float PSO::getVelocity(float time, float A, float B, float C, float D)
    {
        float velocity = A * std::pow(time, 3) + B * std::pow(time, 2) + C * time + D;
        // DLOG(INFO) << "velocity is " << velocity << " time is " << time << " A is " << A << " B is " << B << " C is " << C << " D is " << D;
        DLOG_IF(INFO, velocity < 1e-3) << "velocity is " << velocity << " time is " << time << " A is " << A << " B is " << B << " C is " << C << " D is " << D;
        return velocity;
    }

    std::vector<std::pair<float, float>> PSO::findJerk()
    {
        return findJerkPair(global_best_);
    }

    std::vector<std::pair<float, float>> PSO::findJerkPair(const Particle &particle)
    {
        // DLOG(INFO) << "in findJerk";
        std::vector<std::pair<float, float>> jerk_pair_vec;
        // acceleration=velocity / time, velocity should use average velocity, time = distance / average velocity
        float total_distance, jerk, time_limit, distance = 0, start_velocity, end_velocity, A, B, C, D;
        for (size_t i = 0; i < divided_path_.size(); i++)
        {
            // set velocity
            start_velocity = particle.position_vec[i];
            end_velocity = particle.position_vec[i + 1];
            // set distance
            total_distance = PlannerHelpers::getDistance(divided_path_[i]);
            // set time: t1=2s/(v0+v1)
            if ((start_velocity + end_velocity) != 0)
            {
                time_limit = 2 * total_distance / (start_velocity + end_velocity);
            }
            else
            {
                DLOG(FATAL) << "start velocity + end velocity is zero!!!";
            }

            // A=-2*(v1-v0)/t1^3;B=3(v1-v0)/t1^2;C=0;D=v0
            A = -2 * (end_velocity - start_velocity) / std::pow(time_limit, 3);
            B = 3 * (end_velocity - start_velocity) / std::pow(time_limit, 2);
            C = 0;
            D = start_velocity;
            float previous_distance = 0;
            for (size_t j = 0; j < i; j++)
            {
                previous_distance = previous_distance + PlannerHelpers::getDistance(divided_path_[j]);
            }

            // DLOG(INFO) << "time limit is " << time_limit;
            for (float time = 0; time < time_limit; time = time + 0.01 * time_limit)
            {
                jerk = 6 * A * time + 2 * B;
                distance = 0.25 * A * std::pow(time, 4) + B * std::pow(time, 3) / 3 + D * time;
                distance = distance + previous_distance;
                jerk_pair_vec.emplace_back(std::make_pair(distance, jerk));
                // DLOG(INFO) << "time is " << time;
            }
        }
        // DLOG(INFO) << "out findJerk";
        return jerk_pair_vec;
    }
}
