#include "LpfThreshold.h"

LpfThreshold::LpfThreshold(void) {}

void LpfThreshold::setValues(double alpha, double beta, int jointNumber) {
    alpha_ = alpha;
    beta_ = beta;
    jointNumber_ = jointNumber;
    filtered_signal_b_ = 0.0;
    filtered_signal_ = Eigen::VectorXd::Zero(jointNumber_);
}

double LpfThreshold::adaptiveThreshold(double newSignal, bool high) {
    filtered_signal_b_ = beta_ * newSignal + (1 - beta_) * filtered_signal_b_;
    return high ? filtered_signal_b_ + alpha_: filtered_signal_b_ - alpha_;
}

Eigen::VectorXd LpfThreshold::adaptiveThreshold(const Eigen::VectorXd& newSignal, bool high) {
    filtered_signal_ = beta_ * newSignal + (1 - beta_) * filtered_signal_;
    if(high)
    {
        return filtered_signal_ + Eigen::VectorXd::Constant(jointNumber_, alpha_);
    }
    else
    {
        return filtered_signal_ - Eigen::VectorXd::Constant(jointNumber_, alpha_);
    }
}