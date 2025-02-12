#pragma once

#include <Eigen/Dense>
#include <deque>

class CusumThreshold {
public:
    CusumThreshold(void);

    void setValues(double k, int jointNumber);

    double sensitivityThreshold;
    int jointNumber_;

    std::deque<double> window_d;
    double adaptiveThreshold(double prevThreshold, double newSignal, bool high);

    Eigen::VectorXd adaptiveThreshold(const Eigen::VectorXd& prevThreshold, const Eigen::VectorXd& newSignal, bool high);

    void resetCusum(void);

private:
    double computeCusum(double& cusum, double newSignal, double mean, bool high);
    Eigen::VectorXd computeCusum(Eigen::VectorXd& cusum, const Eigen::VectorXd& newSignal, const Eigen::VectorXd& mean, bool high);

    double cusum_pos_d;
    double cusum_neg_d;
    Eigen::VectorXd cusum_pos;
    Eigen::VectorXd cusum_neg;
};
