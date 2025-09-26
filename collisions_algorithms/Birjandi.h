#pragma once
#include <mc_control/GlobalPlugin.h>
#include <mc_observers/Observer.h>
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <state-observation/observer/extended-kalman-filter.hpp>
#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include "BirjandiStateDynamics.h"
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>
#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

class Birjandi {

public:
    Birjandi(void);

    void init(mc_control::MCGlobalController & ctl, std::string imuBodyName, std::string jointBodyName, std::string robotBodyName);
    void updateFilter(mc_control::MCGlobalController & ctl);
    void collisionDetection(mc_control::MCGlobalController & ctl);
    void computeTauExtRealRobot(mc_control::MCGlobalController & ctl);
    void computeTauExtInputRobot(mc_control::MCGlobalController & ctl);
    void addPlot(std::vector<std::string> & plots, double counter, mc_control::MCGlobalController & ctl);
    void addLog(mc_control::MCGlobalController & ctl);
    void updateCounter(double counter);
    

private:
    std::string imuBodyName_;
    std::string jointBodyName_;
    std::string robotBodyName_;
    bool imu_not_yet_initialized_;
    int jointNumber_;
    double dt_;
    double counter_;

    // Measurements
    Eigen::Vector3d accelero_;
    Eigen::Vector3d gyro_;
    Eigen::Vector3d acclin_joint_;
    Eigen::Vector3d angvel_joint_;
    double q_;

    // Inputs
    Eigen::Vector3d angveldot_;

    // Estimations
    double qdot_hat_;
    double qddot_hat_;

    // Ground truth (derived from encoders)
    double qdot_;
    double qddot_;
    double distance_imu_joint_ = 0.312685;
    double a_filter_ = 0.1;
    double qddot_qp_;
    double qddot_accelero_;

    // stateObservation::kine::Orientation R_joint_;
    Eigen::Matrix3d R_joint_;

    // EKF filter
    stateObservation::ExtendedKalmanFilter filter_;
    stateObservation::BirjandiStateDynamics stateDynamics_;

    // Covariances
    Eigen::MatrixXd r_covariance_;
    Eigen::MatrixXd q_covariance_;
    double qCovariance_ = 1e-8;
    double acceleroCovariance_ = 0.3;
    double gyroCovariance_ = 1e-8;
    double stateCov = 3e-9;
    double stateInitCov = 1e-8;
    

    // Real robot
    rbd::ForwardDynamics forwardDynamics_;
    Eigen::MatrixXd inertiaMatrix_;
    Eigen::MatrixXd coriolisMatrix_;
    Eigen::VectorXd tau_ext_;
    
    // Robot with EKF estimation
    rbd::ForwardDynamics forwardDynamics_hat_;
    Eigen::MatrixXd inertiaMatrix_hat_;
    Eigen::MatrixXd coriolisMatrix_hat_;
    Eigen::VectorXd tau_ext_hat_;
    Eigen::Vector3d ya_; //measure dynamics
    

    
    Eigen::VectorXd tau_fric;
    Eigen::VectorXd tau_m;
    double gear_ratio = 100.0;
    std::map<std::string, double> kt;

    std::shared_ptr<mc_rbdyn::Robots> robot_copied_;

protected:
    /// Sizes of the states for the state, measurement, and input vectors
    static constexpr unsigned STATE_SIZE = 4;
    static constexpr unsigned MEASUREMENT_SIZE = 7;
    static constexpr unsigned INPUT_SIZE = 0;
};
