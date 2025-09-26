#pragma once

#include <mc_control/GlobalPlugin.h>

#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

class SuperTwisting {
    
public:
    SuperTwisting(void);
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
    double gain_gamma2 = 100.0; //γ2
    double gain_gamma1 = 21; //γ1
    double gain_gamma3 = 0; //γ3 //100 (adaptive)
    double gain_gamma4 = 0; //γ4 //100 (adaptive)
    Eigen::MatrixXd gamma_1;
    Eigen::MatrixXd gamma_2;
    Eigen::MatrixXd gamma_3;
    Eigen::MatrixXd gamma_4;

    double counter_;
};