#include "mpc.h"

namespace adaptive_open_local_planner
{
    // good
    void MPC::initialize(const double &vehicle_length, const double &dt, const double &delay, const int &predict_length, const double &heading_weighting, const double &last_heading_weighting, const double &speed_weighting, const double &v_max, const double &a_max, const double &steering_angle_max, const double &steering_angle_rate_max, bool evaluate_path)
    {
        // DLOG(INFO) << "initialize";
        // vehicle length
        ll_ = vehicle_length;
        // delta time
        dt_ = dt;
        // time delay for state x0
        delay_ = delay;
        // should be predicted length
        N_ = predict_length;
        heading_weighting_ = heading_weighting;
        last_heading_weighting_ = last_heading_weighting;
        speed_weighting_ = speed_weighting;
        v_max_ = v_max;
        a_max_ = a_max;
        steering_angle_max_ = steering_angle_max;
        steering_angle_rate_max_ = steering_angle_rate_max;
        evaluate_path_ = evaluate_path;
        if (evaluate_path_)
        {
            std::string position_error_topic, heading_error_topic, velocity_error_topic;
            position_error_topic = "position_error";
            heading_error_topic = "heading_error";
            velocity_error_topic = "velocity_error";

            position_error_pub = nh.advertise<std_msgs::Float32>(position_error_topic, 100, true);

            heading_error_pub = nh.advertise<std_msgs::Float32>(heading_error_topic, 100, true);

            velocity_error_pub = nh.advertise<std_msgs::Float32>(velocity_error_topic, 100, true);
        }

        // DLOG(INFO) << "MPC initialized. vehicle length is " << ll_ << " delta time is " << dt_ << " time delay is " << delay_ << " predicted length is " << N_ << " heading_weighting is " << heading_weighting_ << " last_heading_weighting_heading_weighting is " << last_heading_weighting_ << " max velocity is " << v_max_ << " max acceleration is " << a_max_ << " max steering angle is " << steering_angle_max_ << " max steering_angle_rate_max_ is " << steering_angle_rate_max_;
    }
    // good
    bool MPC::inputRefTrajectory(const std::vector<VectorX> &ref_trajectory)
    {
        // DLOG(INFO) << "inputRefTrajectory";
        ref_trajectory_ = ref_trajectory;
        // for (const auto &element : ref_trajectory_)
        // {
        //     DLOG(INFO) << "ref element is " << element[0] << " " << element[1] << " heading is " << element[2] << " speed is " << element[3];
        // }
        if (N_ > ref_trajectory_.size())
        {
            DLOG(INFO) << "predict length " << N_ << " is larger than ref trajectory size!!" << ref_trajectory_.size();
            N_ = ref_trajectory_.size();
        }

        return true;
    }
    // good
    VectorX MPC::findNext(const VectorX &x0_observe)
    {
        // DLOG(INFO) << "findNext";
        if (ref_trajectory_.size() <= 0)
        {
            DLOG(WARNING) << "ref trajectory not set!";
        }
        int index = findClosestIndex(ref_trajectory_, x0_observe);
        VectorX next;
        if (index < ref_trajectory_.size() - 1)
        {
            next = ref_trajectory_[index + 1];
        }
        else
        {
            next = ref_trajectory_[index];
        }
        // DLOG(INFO) << "current position is " << x0_observe[0] << " " << x0_observe[1] << " heading is " << x0_observe[2] << " speed is " << x0_observe[3];
        // DLOG(INFO) << "next position is " << next[0] << " " << next[1] << " heading is " << next[2] << " speed is " << next[3];
        return next;
    }

    VectorU MPC::output(const VectorX &x0_observe)
    {
        // DLOG(INFO) << "in output";
        VectorU output;
        // DLOG(INFO) << "current position is " << x0_observe[0] << " " << x0_observe[1] << " heading is " << x0_observe[2] << " speed is " << x0_observe[3];
        VectorX x1 = findNext(x0_observe);
        int status = solveMPC(x1);
        if (status != 1)
        {
            DLOG(INFO) << "solve MPC failed!!!";
            return output;
        }
        else
        {
            output = predictInput_.front();
        }
        return output;
    }

    // good
    void MPC::setWeighting()
    {
        // TODO change weighting factor
        // DLOG(INFO) << "in setWeighting";
        // set size of sparse matrices
        Qx_.resize(number_of_state_ * N_, number_of_state_ * N_);
        // stage cost
        Qx_.setIdentity();
        for (int i = 1; i < N_; ++i)
        {
            // heading weighting
            Qx_.coeffRef(i * number_of_state_ - 2, i * number_of_state_ - 2) = heading_weighting_;
            // speed weighting
            Qx_.coeffRef(i * number_of_state_ - 1, i * number_of_state_ - 1) = speed_weighting_;
        }
        // position weighting
        Qx_.coeffRef(N_ * number_of_state_ - 4, N_ * number_of_state_ - 4) = last_heading_weighting_;
        Qx_.coeffRef(N_ * number_of_state_ - 3, N_ * number_of_state_ - 3) = last_heading_weighting_;
        // heading weighting
        Qx_.coeffRef(N_ * number_of_state_ - 2, N_ * number_of_state_ - 2) = last_heading_weighting_ * heading_weighting_;
        // DLOG(INFO) << Qx_;
    }
    // looks good
    void MPC::linearization(const double &phi, const double &v, const double &steering_angle)
    {
        // DLOG(INFO) << "linearization";
        // DLOG(INFO) << "heading is " << phi << " velocity is " << v << " steering angle is " << steering_angle;
        Ad_ << 0, 0, -v * sin(phi), cos(phi),
            0, 0, v * cos(phi), sin(phi),
            0, 0, 0, tan(steering_angle) / ll_,
            0, 0, 0, 0;
        Bd_ << 0, 0,
            0, 0,
            0, v / (ll_ * pow(cos(steering_angle), 2)),
            1, 0;
        gd_ << v * phi * sin(phi), -v * phi * cos(phi), -v * steering_angle / (ll_ * pow(cos(steering_angle), 2)), 0;
        Ad_ = MatrixA::Identity() + dt_ * Ad_;
        Bd_ = dt_ * Bd_;
        gd_ = dt_ * gd_;
        // DLOG(INFO) << "Ad_ is " << Ad_;
        // DLOG(INFO) << "Bd_ is " << Bd_;
        // DLOG(INFO) << "gd_ is " << gd_;
        return;
    }
    // looks good
    std::vector<Eigen::MatrixXd> MPC::setupBBAAggmatrix(const int &predicted_length, const VectorX &x0)
    {
        // DLOG(INFO) << "setupBBAAggmatrix";
        std::vector<Eigen::MatrixXd> result;
        Eigen::MatrixXd BB, AA, gg;
        BB.setZero(number_of_state_ * predicted_length, number_of_control_ * predicted_length);
        AA.setZero(number_of_state_ * predicted_length, number_of_state_);
        gg.setZero(number_of_state_ * predicted_length, 1);
        // s0 is trajectory length
        double s0 = findtrajetorylength(ref_trajectory_, x0);
        double total_length = findtrajetorylength(ref_trajectory_, ref_trajectory_.back());
        double last_phi = x0(2), x, y, phi;
        for (int i = 0; i < predicted_length; ++i)
        {
            updateAdBdgd(s0, x, y, last_phi, phi);
            // calculate big state-space matrices
            /* *                BB                AA
             * x1    /       B    0  ... 0 \    /   A \
             * x2    |      AB    B  ... 0 |    |  A2 |
             * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
             * ...   |     ...  ...  ... 0 |    | ... |
             * xN    \A^(n-1)B  ...  ... B /    \ A^N /
             *
             *     X = BB * U + AA * x0 + gg
             * */
            if (i == 0)
            {
                BB.block(0, 0, number_of_state_, number_of_control_) = Bd_;
                AA.block(0, 0, number_of_state_, number_of_state_) = Ad_;
                gg.block(0, 0, number_of_state_, 1) = gd_;
            }
            else
            {
                BB.block(i * number_of_state_, i * number_of_control_, number_of_state_, number_of_control_) = Bd_;
                for (int j = i - 1; j >= 0; --j)
                {
                    BB.block(i * number_of_state_, j * number_of_control_, number_of_state_, number_of_control_) = Ad_ * BB.block((i - 1) * number_of_state_, j * number_of_control_, number_of_state_, number_of_control_);
                }
                AA.block(i * number_of_state_, 0, number_of_state_, number_of_state_) = Ad_ * AA.block((i - 1) * number_of_state_, 0, number_of_state_, number_of_state_);
                gg.block(i * number_of_state_, 0, number_of_state_, 1) = Ad_ * gg.block((i - 1) * number_of_state_, 0, number_of_state_, 1) + gd_;
            }
            s0 += desired_v_ * dt_;

            s0 = s0 < total_length ? s0 : total_length;
            // DLOG(INFO) << "s0 is " << s0 << " desired_v_ is " << desired_v_ << " dt is " << dt_;
        }
        result.emplace_back(BB);
        result.emplace_back(AA);
        result.emplace_back(gg);
        return result;
    }
    // looks good
    VectorX MPC::compensateDelay(const VectorX &x0)
    {
        // DLOG(INFO) << "compensateDelay";
        VectorX x0_delay = x0;
        if (delay_ == 0)
        {
            DLOG(INFO) << "x0_delay is " << x0_delay[0] << " " << x0_delay[1] << " heading is " << x0_delay[2] << " speed is " << x0_delay[3];
            return x0_delay;
        }
        if (historyInput_.size() <= 0)
        {
            DLOG(INFO) << "historyInput_ size is zero!!!";
            DLOG(INFO) << "x0_delay is " << x0_delay[0] << " " << x0_delay[1] << " heading is " << x0_delay[2] << " speed is " << x0_delay[3];
            return x0_delay;
        }

        Eigen::MatrixXd BB, AA, gg, x0_pred;
        int tau = std::ceil(delay_ / dt_);
        BB.setZero(number_of_state_ * tau, number_of_control_ * tau);
        AA.setZero(number_of_state_ * tau, number_of_state_);
        gg.setZero(number_of_state_ * tau, 1);
        x0_pred.setZero(number_of_state_ * tau, 1);
        std::vector<Eigen::MatrixXd> matrix_vec = setupBBAAggmatrix(tau, x0);
        BB = matrix_vec[0];
        AA = matrix_vec[1];
        gg = matrix_vec[2];
        Eigen::VectorXd uu(number_of_control_ * tau, 1);
        for (double t = delay_; t > 0; t -= dt_)
        {
            int i = std::ceil(t / dt_);
            uu.coeffRef((tau - i) * number_of_control_ + 0, 0) = historyInput_[i - 1][0];
            uu.coeffRef((tau - i) * number_of_control_ + 1, 0) = historyInput_[i - 1][1];
        }
        x0_pred = BB * uu + AA * x0 + gg;
        x0_delay = x0_pred.block((tau - 1) * number_of_state_, 0, number_of_state_, 1);
        // DLOG(INFO) << "x0_delay is " << x0_delay[0] << " " << x0_delay[1] << " heading is " << x0_delay[2] << " speed is " << x0_delay[3] << " x0 is " << x0[0] << " " << x0[1] << " heading is " << x0[2] << " speed is " << x0[3];
        return x0_delay;
    }

    Eigen::SparseMatrix<double> MPC::setupqx(const VectorX &x0)
    {
        // DLOG(INFO) << "setupqx";
        // DLOG(INFO) << "x0 is " << x0(0) << " " << x0(1) << " " << x0(2) << " " << x0(3);
        Eigen::SparseMatrix<double> qx;
        qx.resize(number_of_state_ * N_, 1);
        double s0 = findtrajetorylength(ref_trajectory_, x0);
        double total_length = findtrajetorylength(ref_trajectory_, ref_trajectory_.back());
        double x, y, last_phi = x0(2), phi;
        for (int i = 0; i < N_; ++i)
        {
            updateAdBdgd(s0, x, y, last_phi, phi);
            // cost function should be represented as follows:
            /* *
             *           /  x1  \T       /  x1  \         /  x1  \
             *           |  x2  |        |  x2  |         |  x2  |
             *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
             *           | ...  |        | ...  |         | ...  |
             *           \  xN  /        \  xN  /         \  xN  /
             *
             * qx=-Qx_T * X_ref.
             * */

            qx.coeffRef(i * number_of_state_ + 0, 0) = -Qx_.coeffRef(i * number_of_state_ + 0, i * number_of_state_ + 0) * x;
            qx.coeffRef(i * number_of_state_ + 1, 0) = -Qx_.coeffRef(i * number_of_state_ + 1, i * number_of_state_ + 1) * y;
            qx.coeffRef(i * number_of_state_ + 2, 0) = -Qx_.coeffRef(i * number_of_state_ + 2, i * number_of_state_ + 2) * phi;
            qx.coeffRef(i * number_of_state_ + 3, 0) = -Qx_.coeffRef(i * number_of_state_ + 3, i * number_of_state_ + 3) * desired_v_;

            // DLOG(INFO) << "next point on ref trajectory is " << x << " " << y << " " << phi << " " << desired_v_;
            s0 += desired_v_ * dt_;

            s0 = s0 < total_length ? s0 : total_length;
            // DLOG(INFO) << "s0 is " << s0 << " desired_v_ is " << desired_v_ << " dt is " << dt_;
        }
        return qx;
    }
    // good
    void MPC::findPoint(const double &distance, double &x, double &y, double &heading, double &speed)
    {
        // DLOG(INFO) << "findPoint";
        VectorX point = interpolate(distance);
        x = point[0];
        y = point[1];
        heading = point[2];
        speed = point[3];
        // DLOG(INFO) << " ref trajectory size is " << ref_trajectory_.size() << " distance is " << distance << " " << x << " " << y << " " << heading << " " << speed;
    }
    //  good
    int MPC::findClosestIndex(const std::vector<VectorX> &ref_trajectory, const double &distance)
    {
        // DLOG(INFO) << "in findClosestIndex";
        int index = 0;
        double path_length_pre, path_length_succ;

        if (distance <= 0)
        {
            // DLOG(INFO) << "index is " << index;
            return index;
        }

        for (size_t i = 0; i < ref_trajectory.size() - 1; i++)
        {
            path_length_pre = findLength(ref_trajectory, i);
            if (distance <= path_length_pre)
            {
                index = i;
                break;
            }

            path_length_succ = findLength(ref_trajectory, i + 1);
            if (distance > path_length_pre && distance < path_length_succ)
            {
                index = i;
                break;
            }
            // DLOG(INFO) << i << "th pre length is " << path_length_pre << " succ length is " << path_length_succ;
        }
        // DLOG(INFO) << "distance is " << distance << " index is " << index;
        // if (index == 0)
        // {
        //     DLOG(WARNING) << "index equal to zero!!!Impossible";
        // }
        // DLOG(INFO) << "out findClosestIndex";
        return index;
    }
    // looks good
    void MPC::updateAdBdgd(const double &arc_length, double &x, double &y, double &last_phi, double &phi)
    {
        // DLOG(INFO) << "updateAdBdgd";
        // DLOG(INFO) << "arc length is " << arc_length << " x is " << x << " y is " << y << " last heading is " << last_phi << " current phi is " << phi;
        double steering_angle;
        findPoint(arc_length, x, y, phi, desired_v_);
        if (phi - last_phi > M_PI)
        {
            phi -= 2 * M_PI;
        }
        else if (phi - last_phi < -M_PI)
        {
            phi += 2 * M_PI;
        }
        steering_angle = phi - last_phi;
        last_phi = phi;
        linearization(phi, desired_v_, steering_angle);
    }

    // looks good
    Eigen::SparseMatrix<double> MPC::setupHessian()
    {
        // DLOG(INFO) << "setupHessian";
        Eigen::SparseMatrix<double> hessian;
        hessian.resize(number_of_control_ * N_, number_of_control_ * N_);
        Eigen::SparseMatrix<double> BB_sparse = BB_.sparseView();
        Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
        hessian = BBT_sparse * Qx_ * BB_sparse;
        return hessian;
    }
    // looks good
    Eigen::VectorXd MPC::setupGradient(const VectorX &x0, const Eigen::SparseMatrix<double> &qx)
    {
        // DLOG(INFO) << "setupGradient";
        // gradient= BB_T * Qx_T *(AA*X+gg)+B_T*qx
        Eigen::SparseMatrix<double> gradient;
        gradient.resize(number_of_control_ * N_, 1);
        Eigen::SparseMatrix<double> BB_sparse = BB_.sparseView();
        Eigen::SparseMatrix<double> AA_sparse = AA_.sparseView();
        Eigen::SparseMatrix<double> gg_sparse = gg_.sparseView();

        Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();
        Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
        gradient = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;
        // osqp
        Eigen::VectorXd q_d = gradient.toDense();
        return q_d;
    }
    // looks good
    std::vector<Eigen::SparseMatrix<double>> MPC::setStateConstrain()
    {
        // DLOG(INFO) << "setStateConstrain";
        std::vector<Eigen::SparseMatrix<double>> state_constrain_vec;
        Eigen::SparseMatrix<double> lx, Cx, ux;
        Cx.resize(1 * N_, number_of_state_ * N_);
        lx.resize(1 * N_, 1);
        ux.resize(1 * N_, 1);
        for (int i = 0; i < N_; ++i)
        {
            // -v_max_ <= v <= v_max_
            Cx.coeffRef(i, i * number_of_state_ + 3) = 1;
            lx.coeffRef(i, 0) = 0;
            ux.coeffRef(i, 0) = v_max_;
        }
        state_constrain_vec.emplace_back(Cx);
        state_constrain_vec.emplace_back(lx);
        state_constrain_vec.emplace_back(ux);
        return state_constrain_vec;
    }
    // looks good
    std::vector<Eigen::SparseMatrix<double>> MPC::setControlConstrain()
    {
        // DLOG(INFO) << "setControlConstrain";
        std::vector<Eigen::SparseMatrix<double>> control_constrain_vec;
        Eigen::SparseMatrix<double> lu, Cu, uu;
        // a delta constrains
        Cu.resize(3 * N_, number_of_control_ * N_);
        lu.resize(3 * N_, 1);
        uu.resize(3 * N_, 1);
        // set lower and upper boundaries
        for (int i = 0; i < N_; ++i)
        {
            // -a_max_ <= a <= a_max_ for instance:
            Cu.coeffRef(i * 3 + 0, i * number_of_control_ + 0) = 1;
            lu.coeffRef(i * 3 + 0, 0) = -a_max_;
            uu.coeffRef(i * 3 + 0, 0) = a_max_;
            // min steering angle<=steering angle<=max steering angle
            Cu.coeffRef(i * 3 + 1, i * number_of_control_ + 1) = 1;
            lu.coeffRef(i * 3 + 1, 0) = -steering_angle_max_;
            uu.coeffRef(i * 3 + 1, 0) = steering_angle_max_;
            // delta steering angle smaller than limit
            Cu.coeffRef(i * 3 + 2, i * number_of_control_ + 1) = 1;
            if (i > 0)
            {
                Cu.coeffRef(i * 3 + 2, (i - 1) * number_of_control_ + 1) = -1;
            }
            if (i == 0)
            {
                // lu_.coeffRef(2, 0) = predictInput_.front()(1) - steering_angle_rate_max_ * dt_;
                // uu_.coeffRef(2, 0) = predictInput_.front()(1) + steering_angle_rate_max_ * dt_;
                lu.coeffRef(2, 0) = -steering_angle_max_;
                uu.coeffRef(2, 0) = +steering_angle_max_;
            }
            else
            {
                lu.coeffRef(i * 3 + 2, 0) = -steering_angle_rate_max_ * dt_;
                uu.coeffRef(i * 3 + 2, 0) = steering_angle_rate_max_ * dt_;
            }
        }
        control_constrain_vec.emplace_back(Cu);
        control_constrain_vec.emplace_back(lu);
        control_constrain_vec.emplace_back(uu);
        return control_constrain_vec;
    }
    // looks good
    void MPC::combineConstrain(const std::vector<Eigen::SparseMatrix<double>> &state_constrain, const std::vector<Eigen::SparseMatrix<double>> &control_constrain, const Eigen::SparseMatrix<double> &x0_sparse, const Eigen::SparseMatrix<double> &BB_sparse, const Eigen::SparseMatrix<double> &AA_sparse, const Eigen::SparseMatrix<double> &gg_sparse, Eigen::SparseMatrix<double> &A,
                               Eigen::VectorXd &lower_limit_d,
                               Eigen::VectorXd &upper_limit_d)
    {
        // DLOG(INFO) << "combineConstrain";
        int n_cons = 4; // v a steering_angle steering_angle_rate
        A.resize(n_cons * N_, number_of_control_ * N_);
        Eigen::SparseMatrix<double> lower_limit, upper_limit;
        lower_limit.resize(n_cons * N_, 1);
        upper_limit.resize(n_cons * N_, 1);

        // state constrants propogate to input constraints using "X = BB * U + AA * x0 + gg"
        /* *
         *               /  x1  \                              /  u0  \
         *               |  x2  |                              |  u1  |
         *  lx_ <=  Cx_  |  x3  |  <= ux_    ==>    lx <=  Cx  |  u2  |  <= ux
         *               | ...  |                              | ...  |
         *               \  xN  /                              \ uN-1 /
         * */
        Eigen::SparseMatrix<double> Cx = state_constrain[0] * BB_sparse;
        Eigen::SparseMatrix<double> lx = state_constrain[1] - state_constrain[0] * AA_sparse * x0_sparse - state_constrain[0] * gg_sparse;
        Eigen::SparseMatrix<double> ux = state_constrain[2] - state_constrain[0] * AA_sparse * x0_sparse - state_constrain[0] * gg_sparse;

        /* *      / Cx  \                / lx  \                / ux  \
         *   A_ = \ Cu_ /, lower_limit = \ lu_ /, upper_limit = \ uu_ /
         * */

        Eigen::SparseMatrix<double> A_T = A.transpose();
        A_T.middleCols(0, Cx.rows()) = Cx.transpose();
        A_T.middleCols(Cx.rows(), control_constrain[0].rows()) = control_constrain[0].transpose();
        A = A_T.transpose();
        for (int i = 0; i < lx.rows(); ++i)
        {
            lower_limit.coeffRef(i, 0) = lx.coeff(i, 0);
            upper_limit.coeffRef(i, 0) = ux.coeff(i, 0);
        }
        for (int i = 0; i < control_constrain[1].rows(); ++i)
        {
            lower_limit.coeffRef(i + lx.rows(), 0) = control_constrain[1].coeff(i, 0);
            upper_limit.coeffRef(i + lx.rows(), 0) = control_constrain[2].coeff(i, 0);
        }

        lower_limit_d = lower_limit.toDense();
        upper_limit_d = upper_limit.toDense();
    }
    // looks good
    int MPC::solveMPC(const VectorX &x0_observe)
    {
        // DLOG(INFO) << "in solveMPC";
        // historyInput_.clear();
        // DLOG(INFO) << "historyInput_ size is " << historyInput_.size();
        // DLOG(INFO) << "predictInput_ size is " << predictInput_.size();
        historyInput_ = predictInput_;
        // DLOG(INFO) << "x0_observe is " << x0_observe(0) << " " << x0_observe(1) << " " << x0_observe(2) << " " << x0_observe(3);
        setWeighting();
        // VectorX x0 = compensateDelay(x0_observe);
        VectorX x0 = x0_observe;
        Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();
        // set BB, AA, gg
        std::vector<Eigen::MatrixXd> matrix_vec = setupBBAAggmatrix(N_, x0);
        BB_ = matrix_vec[0];
        AA_ = matrix_vec[1];
        gg_ = matrix_vec[2];
        Eigen::SparseMatrix<double> qx = setupqx(x0);
        Eigen::SparseMatrix<double> BB_sparse = BB_.sparseView();
        Eigen::SparseMatrix<double> AA_sparse = AA_.sparseView();
        Eigen::SparseMatrix<double> gg_sparse = gg_.sparseView();

        Eigen::SparseMatrix<double> hessian_matrix = setupHessian();
        Eigen::VectorXd gradient_matrix = setupGradient(x0, qx);

        std::vector<Eigen::SparseMatrix<double>> state_constrain_vec = setStateConstrain();
        std::vector<Eigen::SparseMatrix<double>> control_constrain_vec = setControlConstrain();
        Eigen::SparseMatrix<double> A;
        Eigen::VectorXd lower_limit_d, upper_limit_d;
        combineConstrain(state_constrain_vec, control_constrain_vec, x0_sparse, BB_sparse, AA_sparse, gg_sparse, A, lower_limit_d, upper_limit_d);

        int ret = solveQP(hessian_matrix, gradient_matrix, A, lower_limit_d, upper_limit_d, x0);

        if (ret != 1)
        {
            DLOG(INFO) << "fail to solve QP!";
            return ret;
        }

        return ret;
    }
    // looks good
    int MPC::solveQP(const Eigen::SparseMatrix<double> &hessian, const Eigen::Ref<Eigen::VectorXd> &gradient, const Eigen::SparseMatrix<double> &linearMatrix, const Eigen::Ref<Eigen::VectorXd> &lowerBound, const Eigen::Ref<Eigen::VectorXd> &upperBound, const VectorX &x0)
    {
        // DLOG(INFO) << "in solveQP";
        // instantiate the solver
        OsqpEigen::Solver solver;
        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        int n_cons = 4; // v a steering_angle steering_angle_rate
        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(number_of_control_ * N_);
        solver.data()->setNumberOfConstraints(n_cons * N_);
        if (!solver.data()->setHessianMatrix(hessian))
        {
            DLOG(INFO) << "set hessian failed.";
            return 0;
        }

        if (!solver.data()->setGradient(gradient))
        {
            DLOG(INFO) << "set hessian failed.";
            return 0;
        }
        if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        {
            DLOG(INFO) << "set LinearConstraintsMatrix failed.";
            return 0;
        }
        if (!solver.data()->setLowerBound(lowerBound))
        {
            DLOG(INFO) << "set LowerBound failed.";
            return 0;
        }
        if (!solver.data()->setUpperBound(upperBound))
        {
            DLOG(INFO) << "set UpperBound failed.";
            return 0;
        }

        // instantiate the solver
        if (!solver.initSolver())
        {
            DLOG(INFO) << "initSolver failed.";
            return 0;
        }

        // solve the QP problem
        if (!solver.solve())
        {
            DLOG(INFO) << "solve QP failed.";
            return 0;
        }
        // DLOG(INFO) << "solve QP success.";
        Eigen::VectorXd sol = solver.getSolution();
        // DLOG(INFO) << "solve QP success.";
        Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), number_of_control_, N_);
        // DLOG(INFO) << "solve QP success.";
        Eigen::VectorXd solState = BB_ * sol + AA_ * x0 + gg_;
        // DLOG(INFO) << "solve QP success.";
        predictMat_ = Eigen::Map<const Eigen::MatrixXd>(solState.data(), number_of_state_, N_);
        // seems meaningless
        // double cost = findCost(hessian, sol, gradient);
        // cost_vec_.emplace_back(cost);
        // DLOG(INFO) << "solve QP success.";
        // DLOG(INFO) << "predictInput_ size is " << predictInput_.size();
        if (evaluate_path_)
        {
            std::vector<double> error_vec = findError(predictMat_);
            // DLOG(INFO) << "position error is " << error_vec[0];
            // DLOG(INFO) << "heading error is " << error_vec[1];
            // DLOG(INFO) << "velocity error is " << error_vec[2];
            publishErrors(error_vec);
        }

        predictInput_.clear();
        for (int i = 0; i < N_; ++i)
        {
            predictInput_.emplace_back(solMat.col(i));
            // predictInput_[i] = solMat.col(i);
            // DLOG(INFO) << i << "th predicted input is: acceleration is " << solMat.col(i)[0] << " steering angle is " << solMat.col(i)[1];
            // DLOG(INFO) << i << "th predicted state is " << predictMat_.col(i)[0] << " " << predictMat_.col(i)[1] << " heading is " << predictMat_.col(i)[2] << " speed is " << predictMat_.col(i)[3];
            // DLOG(INFO) << "ref state is " << ref_trajectory_[i][0] << " " << ref_trajectory_[i][1] << " heading is " << ref_trajectory_[i][2] << " speed is " << ref_trajectory_[i][3];
        }

        // DLOG(INFO) << "solve QP success.";
        return 1;
    }
    //  good
    double MPC::findtrajetorylength(const std::vector<VectorX> &ref_trajectory, const VectorX &current_location)
    {
        // DLOG(INFO) << "findtrajetorylength";
        double length;
        int closest_index = findClosestIndex(ref_trajectory, current_location);
        length = findLength(ref_trajectory, closest_index);
        // DLOG(INFO) << "current location is " << current_location[0] << " " << current_location[1] << " closest index is " << closest_index << " length is " << length;
        return length;
    }
    // good
    int MPC::findClosestIndex(const std::vector<VectorX> &ref_trajectory, const VectorX &current_location)
    {
        // DLOG(INFO) << "findClosestIndex";
        int index;
        auto it = std::find(ref_trajectory.begin(), ref_trajectory.end(), current_location);

        if (it != ref_trajectory.end())
        {
            index = it - ref_trajectory.begin();
        }
        else
        {
            double min_distance = 100000, distance;
            for (size_t i = 0; i < ref_trajectory.size(); i++)
            {
                distance = findDistance(ref_trajectory[i], current_location);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    index = i;
                }
            }
        }
        // DLOG(INFO) << "current location is " << current_location[0] << " " << current_location[1];
        // int i = 0;
        // for (const auto &element : ref_trajectory)
        // {
        //     DLOG(INFO) << i << "th ref element is " << element[0] << " " << element[1] << " heading is " << element[2] << " speed is " << element[3];
        //     i++;
        // }
        return index;
    }
    //  good
    double MPC::findDistance(const VectorX &p1, const VectorX &p2)
    {
        // DLOG(INFO) << "findDistance";
        double result;
        result = std::sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
        // DLOG(INFO) << "p1 is " << p1[0] << " " << p1[1];
        // DLOG(INFO) << "p2 is " << p2[0] << " " << p2[1];
        // DLOG(INFO) << "distance is " << result;
        return result;
    }
    // good
    double MPC::findLength(const std::vector<VectorX> &ref_trajectory, const int &closest_index)
    {
        // DLOG(INFO) << "findLength";
        double length = 0;
        for (size_t i = 0; i < closest_index; i++)
        {
            length += findDistance(ref_trajectory[i], ref_trajectory[i + 1]);
        }
        // DLOG(INFO) << "closest_index is " << closest_index << " " << length;
        return length;
    }
    // good
    VectorX MPC::interpolate(const float &distance)
    {
        // DLOG(INFO) << "interpolate";
        VectorX point;
        if (distance <= 0)
        {
            DLOG(INFO) << "distance : " << distance << " is smaller than 0!!!";
            return ref_trajectory_.front();
        }

        if (ref_trajectory_.size() <= 0)
        {
            DLOG(WARNING) << "ref trajectory not set!";
        }
        double total_length = findtrajetorylength(ref_trajectory_, ref_trajectory_.back());
        double new_distance = 0, path_length_pre, path_length_succ;
        if (distance > total_length)
        {
            // DLOG(INFO) << "distance : " << distance << " is larger than total length : " << total_length;
            point = ref_trajectory_.back();
            return point;
        }
        int index = 0;
        for (size_t i = 0; i < ref_trajectory_.size() - 1; i++)
        {
            path_length_pre = findLength(ref_trajectory_, i);
            path_length_succ = findLength(ref_trajectory_, i + 1);
            if (distance > path_length_pre && distance < path_length_succ)
            {
                index = i;
                new_distance = distance - path_length_pre;
                break;
            }
        }
        point = interpolate(ref_trajectory_[index], ref_trajectory_[index + 1], new_distance);
        return point;
    }
    // good
    VectorX MPC::interpolate(const VectorX &x0, const VectorX &x1, const double &distance)
    {
        // DLOG(INFO) << "interpolate";
        VectorX point;
        double direction, total_distance = findDistance(x0, x1);
        if (x1[0] == x0[0])
        {
            direction = 3.14 * 0.5;
        }
        else
        {
            direction = atan2(x1[1] - x0[1], x1[0] - x0[0]);
        }
        point[0] = x0[0] + distance * cos(direction);
        point[1] = x0[1] + distance * sin(direction);

        double percentage = distance / total_distance;

        // heading
        point[2] = x0[2] + percentage * (x1[2] - x0[2]);
        // velocity
        point[3] = x0[3] + percentage * (x1[3] - x0[3]);

        // DLOG(INFO) << "x0 is " << x0[0] << " " << x0[1] << " " << x0[2] << " " << x0[3];
        // DLOG(INFO) << "x1 is " << x1[0] << " " << x1[1] << " " << x1[2] << " " << x1[3];
        // DLOG(INFO) << "distance is " << distance << " total distance is " << total_distance << " percentage is " << percentage;
        // DLOG(INFO) << "point is " << point[0] << " " << point[1] << " " << point[2] << " " << point[3];
        return point;
    }

    double MPC::findCost(const Eigen::SparseMatrix<double> &hessian, const Eigen::VectorXd &control_vec, const Eigen::VectorXd &gradient_matrix)
    {
        // DLOG(INFO) << "findCost";
        auto control_vec_t = control_vec.transpose();
        auto gradient_matrix_t = gradient_matrix.transpose();
        auto cost = 0.5 * control_vec_t * hessian * control_vec + gradient_matrix_t * control_vec;
        // DLOG(INFO) << "hessian size is " << hessian.rows() << " " << hessian.cols();
        // DLOG(INFO) << "control_vec size is " << control_vec.rows() << " " << control_vec.cols();
        // DLOG(INFO) << "gradient_matrix size is " << gradient_matrix.rows() << " " << gradient_matrix.cols();
        // DLOG(INFO) << "cost is " << cost;
        return cost(0);
    }

    std::vector<double> MPC::findError(const Eigen::MatrixXd &predictMat)
    {
        std::vector<double> error_vec;
        double position_error, heading_error, velocity_error;
        position_error = findPositionError(predictMat);
        heading_error = findHeadingError(predictMat);
        velocity_error = findVelocityError(predictMat);
        error_vec.emplace_back(position_error);
        error_vec.emplace_back(heading_error);
        error_vec.emplace_back(velocity_error);
        return error_vec;
    }

    double MPC::findPositionError(const Eigen::MatrixXd &predictMat)
    {
        double position_error;
        for (int index = 0; index < N_; index++)
        {
            position_error += findDistance(ref_trajectory_[index], predictMat.col(index));
        }
        return position_error;
    }

    double MPC::findHeadingError(const Eigen::MatrixXd &predictMat)
    {
        double heading_error;
        for (int index = 0; index < N_; index++)
        {
            heading_error += std::abs(ref_trajectory_[index][2] - predictMat.col(index)[2]);
        }
        return heading_error;
    }

    double MPC::findVelocityError(const Eigen::MatrixXd &predictMat)
    {
        double velocity_error;
        for (int index = 0; index < N_; index++)
        {
            velocity_error += std::abs(ref_trajectory_[index][3] - predictMat.col(index)[3]);
        }
        return velocity_error;
    }

    Eigen::MatrixXd MPC::getPredictedState()
    {
        return predictMat_;
    }

    bool MPC::publishErrors(const std::vector<double> &error_vec)
    {
        // DLOG(INFO) << "publishErrors";

        std_msgs::Float32 position_error, heading_error, velocity_error;

        position_error.data = error_vec[0];
        heading_error.data = error_vec[1];
        velocity_error.data = error_vec[2];
        // DLOG(INFO) << "publishErrors: position error is " << error_vec[0] << " heading error is " << error_vec[1] << " velocity error is " << error_vec[2];
        position_error_pub.publish(position_error);
        heading_error_pub.publish(heading_error);
        velocity_error_pub.publish(velocity_error);

        return true;
    }
}