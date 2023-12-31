#pragma once
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
// osqp-eigen
#include <algorithm>
#include "glog/logging.h"
#include "gflags/gflags.h"
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
namespace adaptive_open_local_planner
{

    typedef Eigen::Matrix<double, 4, 4> MatrixA;
    typedef Eigen::Matrix<double, 4, 2> MatrixB;
    typedef Eigen::Vector4d VectorG;
    typedef Eigen::Vector4d VectorX;
    typedef Eigen::Vector2d VectorU;

    class MPC
    {
    public:
        void initialize(const double &vehicle_length, const double &dt, const double &delay, const int &predict_length, const double &heading_weighting, const double &last_heading_weighting, const double &speed_weighting, const double &v_max, const double &a_max, const double &steering_angle_max, const double &steering_angle_rate_max, bool evaluate_path);

        bool inputRefTrajectory(const std::vector<VectorX> &ref_trajectory);

        VectorU output(const VectorX &x0_observe);

        Eigen::MatrixXd getPredictedState();

    private:
        void findPoint(const double &distance, double &x, double &y, double &heading, double &speed);

        void updateAdBdgd(const double &arc_length, double &x, double &y, double &last_phi, double &phi);
        /**
         * @brief find trajectory length start from first of ref trajectory to current_location
         *
         * @param ref_trajectory
         * @param location
         * @return double
         */
        double findtrajetorylength(const std::vector<VectorX> &ref_trajectory, const VectorX &current_location);

        int findClosestIndex(const std::vector<VectorX> &ref_trajectory, const VectorX &current_location);
        int findClosestIndex(const std::vector<VectorX> &ref_trajectory, const double &distance);

        double findLength(const std::vector<VectorX> &ref_trajectory, const int &closest_index);

        double findDistance(const VectorX &p1, const VectorX &p2);
        /**
         * @brief calculate matrix A,B,g for x_k+1=Ad_*x_k+Bd_*u_k+gd_;
         *
         * @param phi
         * @param v
         * @param delta
         */
        void linearization(const double &phi, const double &v, const double &delta);
        /**
         * @brief consider a delay for state x0
         *
         * @param x0
         * @return VectorX
         */
        VectorX compensateDelay(const VectorX &x0);

        void setWeighting();

        int solveMPC(const VectorX &x0_observe);
        /**
         * @brief just solve QP problem
         *
         * @param hessian
         * @param gradient
         * @param linearMatrix
         * @param lowerBound
         * @param upperBound
         * @return int 0 failed, 1 success
         */
        int solveQP(const Eigen::SparseMatrix<double> &hessian, const Eigen::Ref<Eigen::VectorXd> &gradient, const Eigen::SparseMatrix<double> &linearMatrix, const Eigen::Ref<Eigen::VectorXd> &lowerBound, const Eigen::Ref<Eigen::VectorXd> &upperBound, const VectorX &x0);

        Eigen::SparseMatrix<double> setupHessian();

        Eigen::VectorXd setupGradient(const VectorX &x0_observe, const Eigen::SparseMatrix<double> &qx);
        //
        /**
         * @brief Set the State Constrain object.
         * -v_max_ <= v <= v_max_, for x,y, theta ,there is no constrain
         *size: (N,1) <= (N,4N)*(4N,1)<=(N,1)
         *               /  x1  \
         *               |  x2  |
         *  lx_ <=  Cx_  |  x3  |  <= ux_
         *               | ...  |
         *               \  xN  /
         *
         * @return std::vector<Eigen::MatrixXd> : first is Cx_, second is lx_, last is ux_
         */
        std::vector<Eigen::SparseMatrix<double>> setStateConstrain();
        /**
         * @brief Set the Control Constrain object

               *                  /  u0  \
               *                  |  u1  |
               *       lx <=  Cx  |  u2  |  <= ux
               *                  | ...  |
               *                  \ uN-1 /

         *
         * @return * std::vector<Eigen::MatrixXd>: first is Cx, second is lx, last is ux
         */
        std::vector<Eigen::SparseMatrix<double>> setControlConstrain();
        /**
         * @brief combine state constrain and control constrain, convert to QSQP format
         *
         * @return std::vector<Eigen::MatrixXd> first is Cx, second is lx, last is ux
         */
        void combineConstrain(const std::vector<Eigen::SparseMatrix<double>> &state_constrain, const std::vector<Eigen::SparseMatrix<double>> &control_constrain, const Eigen::SparseMatrix<double> &x0_sparse, const Eigen::SparseMatrix<double> &BB_sparse, const Eigen::SparseMatrix<double> &AA_sparse, const Eigen::SparseMatrix<double> &gg_sparse, Eigen::SparseMatrix<double> &A,
                              Eigen::VectorXd &lower_limit_d,
                              Eigen::VectorXd &upper_limit_d);
        /**
         * @brief set BB, AA, gg matrix
         *                 BB                AA
         * x1    /       B    0  ... 0 \    /   A \
         * x2    |      AB    B  ... 0 |    |  A2 |
         * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
         * ...   |     ...  ...  ... 0 |    | ... |
         * xN    \A^(n-1)B  ...  ... B /    \ A^N /
         *
         *     X = BB * U + AA * x0 + gg
         * first element is BB
         * second is AA
         * last is gg
         *         @return std::vector<Eigen::MatrixXd> first is BB, second is AA, last is gg
         */
        std::vector<Eigen::MatrixXd>
        setupBBAAggmatrix(const int &predicted_length, const VectorX &x0);
        /**
         * @brief      // cost function should be represented as follows:

             *           /  x1  \T       /  x1  \         /  x1  \
             *           |  x2  |        |  x2  |         |  x2  |
             *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
             *           | ...  |        | ...  |         | ...  |
             *           \  xN  /        \  xN  /         \  xN  /

         *
         * @return Eigen::SparseMatrix<double>
         */
        Eigen::SparseMatrix<double> setupqx(const VectorX &x0);

        VectorX findNext(const VectorX &x0_observe);

        VectorX interpolate(const float &distance);

        VectorX interpolate(const VectorX &x0, const VectorX &x1, const double &distance);

        double findCost(const Eigen::SparseMatrix<double> &hessian, const Eigen::VectorXd &control_vec, const Eigen::VectorXd &gradient_matrix);
        /**
         * @brief find position error, heading error and velocity error for predict state.
         *
         * @param predictMat
         * @return std::vector<double> first is position error, second is heading error, last is velocity error
         */
        std::vector<double> findError(const Eigen::MatrixXd &predictMat);

        double findPositionError(const Eigen::MatrixXd &predictMat);
        double findHeadingError(const Eigen::MatrixXd &predictMat);
        double findVelocityError(const Eigen::MatrixXd &predictMat);

        bool publishErrors(const std::vector<double> &error_vec);

    private:
        int number_of_state_ = 4;   // state x y phi v
        int number_of_control_ = 2; // input a delta
        //    x_k+1=Ad_*x_k+Bd_*u_k+gd_;
        MatrixA Ad_;
        MatrixB Bd_;
        VectorG gd_;
        // vehicle length
        double ll_;
        // delta time
        double dt_;
        // time delay for state x0
        double delay_;
        // history control vector u
        std::vector<VectorU> historyInput_;
        std::vector<VectorU> predictInput_;
        // std::vector<Eigen::MatrixXd> predictState_;

        // weighting factor for cost function,
        Eigen::SparseMatrix<double> Qx_;
        // should be predicted length
        int N_;
        // should be reference trajectory
        std::vector<VectorX> ref_trajectory_;
        Eigen::MatrixXd predictMat_;
        double desired_v_;

        Eigen::MatrixXd BB_;
        Eigen::MatrixXd AA_;
        Eigen::MatrixXd gg_;
        // some parameters
        double heading_weighting_;
        double last_heading_weighting_;
        double speed_weighting_;
        double v_max_;
        double a_max_;
        double steering_angle_max_;
        double steering_angle_rate_max_;

        std::vector<double> cost_vec_;

        bool evaluate_path_;

        ros::NodeHandle nh;
        ros::Publisher position_error_pub;
        ros::Publisher heading_error_pub;
        ros::Publisher velocity_error_pub;
    };
}
