#include "SuperTwisting.h"

SuperTwisting::SuperTwisting(void) {}

void SuperTwisting::init(mc_control::MCGlobalController & controller)
{
    auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

    auto & robot = ctl.robot(ctl.robots()[0].name());
    auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
    auto & rjo = robot.refJointOrder();

    dt_ = ctl.timestep();
    counter_ = 0.0;

    jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();

    gamma_1 = sqrt(3*gain_gamma2)* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
    gamma_2 = gain_gamma2* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
    gamma_3 = gain_gamma3* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
    gamma_4 = gain_gamma4* Eigen::MatrixXd::Identity(jointNumber, jointNumber);

    tau_m.setZero(jointNumber);
    tau_ext_hat.setZero(jointNumber);
    tau_ext_hat_dot.setZero(jointNumber);
    inertiaMatrix.resize(jointNumber, jointNumber);

    Eigen::VectorXd qdot(jointNumber);
    for(size_t i = 0; i < jointNumber; i++)
    {
        qdot[i] = robot.alpha()[robot.jointIndexByName(rjo[i])][0];
    }

    coriolis = new rbd::Coriolis(robot.mb());
    forwardDynamics = rbd::ForwardDynamics(robot.mb());

    forwardDynamics.computeH(robot.mb(), robot.mbc());
    inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
    p = inertiaMatrix * qdot;
    p_hat = Eigen::VectorXd::Zero(jointNumber);
    p_error = p - p_hat;
    auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
    auto coriolisGravityTerm = forwardDynamics.C(); //C*qdot + g
    gamma = tau_m + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm;
}

void SuperTwisting::computeMomemtum(mc_control::MCGlobalController & controller)
{
    auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

    if(ctl.robot().encoderVelocities().empty())
    {
        return;
    }

    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

    auto & rjo = realRobot.refJointOrder();

    Eigen::VectorXd qdot(jointNumber);
    rbd::paramToVector(realRobot.alpha(), qdot);
    tau_m = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size());
    // mc_rtc::log::info("[SuperTwisting] qdot: {}, tau_m: {}", qdot, tau_m);
    forwardDynamics.computeC(realRobot.mb(), realRobot.mbc());
    forwardDynamics.computeH(realRobot.mb(), realRobot.mbc());
    auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
    auto coriolisGravityTerm = forwardDynamics.C(); //C*qdot + g
    gamma = tau_m + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm; //gamma = tau_m -g + C^T*qdot
    inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
    // x_hat_dot = Gamma + γ1*sqrt(|x - x_hat|)*Sign(x - x_hat) + d_hat
    Eigen::VectorXd p_hat_dot = gamma + gamma_1*(p_error).cwiseAbs().cwiseSqrt().cwiseProduct(Sign(p_error)) + gamma_4*(p_error) + tau_ext_hat;
    p_hat += p_hat_dot*dt_;

    // d_hat_dot = γ2*Sign(x - x_hat)
    tau_ext_hat_dot = (gamma_2+gamma_3*p_error.cwiseAbs())*Sign(p_error);
    // integrate tau_ext_hat_dot to get tau_ext_hat
    tau_ext_hat += tau_ext_hat_dot*dt_;
    p = inertiaMatrix * qdot;
    p_error = p - p_hat;
}

double SuperTwisting::sign(double x)
{
    if (x>0) return 1;
    else if (x<0) return -1;
    else return 0;
}

Eigen::VectorXd SuperTwisting::Sign(Eigen::VectorXd x)
{
    Eigen::VectorXd y(x.size());
    for (int i=0; i<x.size(); i++)
    {
        y[i] = sign(x[i]);
    }
    return y;
}

void SuperTwisting::addPlot(std::vector<std::string> & plots, double counter, mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    counter_ = counter;
    //Momentum
    gui.addPlot(
        plots[10],
        mc_rtc::gui::plot::X(
            "t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y(
            "p(t)", [this]() { return p[5]; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y(
            "p_hat(t)", [this]() { return p_hat[5]; }, mc_rtc::gui::Color::Green)
        );
    //Momentum error
    gui.addPlot(
        plots[11],
        mc_rtc::gui::plot::X(
            "t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y(
            "p_error(t)", [this]() { return p_error[5]; }, mc_rtc::gui::Color::Red)
        );
        gui.addPlot(
            "Torque estimation",
            mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
            mc_rtc::gui::plot::Y("tau_ext_hat(t)", [this]() { return -tau_ext_hat[5]; }, mc_rtc::gui::Color::Red),
            mc_rtc::gui::plot::Y("tau_m(t)", [this]() { return tau_m[5]; }, mc_rtc::gui::Color::Green)
        );

    gui.addPlot(
        "Gamma",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("gamma(t)", [this]() { return gamma[5]; }, mc_rtc::gui::Color::Red)
    );
}

void SuperTwisting::addGui(mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "gamma2", [this]() { return gain_gamma2; },
            [this](double gain)
            {
                this->gain_gamma2 = gain;
                // gamma_1 = sqrt(3*gain)* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
                gamma_2 = gain* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "gamma1", [this]() { return gain_gamma1; },
            [this](double gain)
            {
                this->gain_gamma1 = gain;
                gamma_1 = gain* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "gamma3", [this]() { return gain_gamma3; },
            [this](double gain)
            {
                this->gain_gamma3 = gain;
                gamma_3 = gain* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
    gui.addElement({"Plugins", "SuperTwisting"},
        mc_rtc::gui::NumberInput(
            "gamma4", [this]() { return gain_gamma4; },
            [this](double gain)
            {
                this->gain_gamma4 = gain;
                gamma_4 = gain* Eigen::MatrixXd::Identity(jointNumber, jointNumber);
                tau_ext_hat.setZero(jointNumber);
                tau_ext_hat_dot.setZero(jointNumber);
                p_hat.setZero(jointNumber);
            }));
}

void SuperTwisting::addLog(mc_control::MCGlobalController & ctl)
{
    ctl.controller().logger().addLogEntry("SuperTwisting_p", [this]() { return p; });
    ctl.controller().logger().addLogEntry("SuperTwisting_p_hat", [this]() { return p_hat; });
    ctl.controller().logger().addLogEntry("SuperTwisting_p_error", [this]() { return p_error; });
    ctl.controller().logger().addLogEntry("SuperTwisting_tau_ext_hat", [this]() { return tau_ext_hat; });
    ctl.controller().logger().addLogEntry("SuperTwisting_tau_ext_hat_dot", [this]() { return tau_ext_hat_dot; });
    ctl.controller().logger().addLogEntry("SuperTwisting_gamma", [this]() { return gamma; });
}

void SuperTwisting::updateCounter(double counter)
{
    counter_ = counter;
}