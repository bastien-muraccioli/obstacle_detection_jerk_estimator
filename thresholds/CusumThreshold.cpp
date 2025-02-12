#include "CusumThreshold.h"

CusumThreshold::CusumThreshold(void) : cusum_pos_d(0.0), cusum_neg_d(0.0) {}

void CusumThreshold::setValues(double k, int jointNumber) {
    sensitivityThreshold = k;
    jointNumber_ = jointNumber;
    cusum_neg_d = 0.0;
    cusum_pos_d = 0.0;
    cusum_pos = Eigen::VectorXd::Zero(jointNumber_);
    cusum_neg = Eigen::VectorXd::Zero(jointNumber_);
}

double CusumThreshold::adaptiveThreshold(double prevThreshold, double newSignal, bool high) {
    double mean = prevThreshold;  // Assume previous threshold as baseline
    if (high) {
        cusum_pos_d = computeCusum(cusum_pos_d, newSignal, mean, true);
        return mean + cusum_pos_d;
    } else {
        cusum_neg_d = computeCusum(cusum_neg_d, newSignal, mean, false);
        return mean - cusum_neg_d;
    }
}

Eigen::VectorXd CusumThreshold::adaptiveThreshold(const Eigen::VectorXd& prevThreshold, const Eigen::VectorXd& newSignal, bool high) {
    Eigen::VectorXd mean = prevThreshold;  // Assume previous threshold as baseline
    if (high) {
        cusum_pos = computeCusum(cusum_pos, newSignal, mean, true);
        return mean + cusum_pos;
    } else {
        cusum_neg = computeCusum(cusum_neg, newSignal, mean, false);
        return mean - cusum_neg;
    }
}

double CusumThreshold::computeCusum(double& cusum, double newSignal, double mean, bool high) {
    double deviation = high ? (newSignal - mean - sensitivityThreshold) : (mean - newSignal - sensitivityThreshold);
    cusum = std::max(0.0, cusum + deviation);
    return cusum;
}

Eigen::VectorXd CusumThreshold::computeCusum(Eigen::VectorXd& cusum, const Eigen::VectorXd& newSignal, const Eigen::VectorXd& mean, bool high) {
    Eigen::VectorXd deviation = high ? (newSignal - mean - Eigen::VectorXd::Constant(jointNumber_, sensitivityThreshold))
                                     : (mean - newSignal - Eigen::VectorXd::Constant(jointNumber_, sensitivityThreshold));
    cusum = (cusum + deviation).cwiseMax(0.0);
    return cusum;
}

void CusumThreshold::resetCusum(void) {
    cusum_pos_d = 0.0;
    cusum_pos.setZero();
    cusum_neg_d = 0.0;
    cusum_neg.setZero();
}
