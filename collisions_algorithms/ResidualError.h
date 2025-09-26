#pragma once

#include <mc_control/GlobalPlugin.h>

#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

class ResidualError {
    
public:
    ResidualError(void);
    void init(mc_control::MCGlobalController & controller);
    double sign(double x);
    Eigen::VectorXd Sign(Eigen::VectorXd x);
    void computeMomemtum(mc_control::MCGlobalController & controller);
    bool collisionDetection(mc_control::MCGlobalController & ctl);
    void addPlot(std::vector<std::string> & plots, double counter, mc_control::MCGlobalController & ctl);
    void addLog(mc_control::MCGlobalController & ctl);
    void addGui(mc_control::MCGlobalController & ctl);
    void updateCounter(double counter);

private:
    double dt_; // Time step
    int jointNumber;
    // Eigen::VectorXd integralTerm;
    rbd::Coriolis * coriolis;
    rbd::ForwardDynamics forwardDynamics;
    Eigen::VectorXd gamma;
    Eigen::MatrixXd inertiaMatrix;
    Eigen::VectorXd p; //momentum
    Eigen::VectorXd p_hat; //Estimated momentum
    Eigen::VectorXd p_error; //Momentum error
    Eigen::VectorXd tau_m;
    Eigen::VectorXd tau_ext_hat; //Estimated external torque
    Eigen::VectorXd tau_ext_hat_dot; //Estimated external torque derivative
    double gain_gamma2 = 10.0; //γ2
    double gain_gamma1; //γ1
    Eigen::MatrixXd gamma_1;
    Eigen::MatrixXd gamma_2;

    double counter_;
};