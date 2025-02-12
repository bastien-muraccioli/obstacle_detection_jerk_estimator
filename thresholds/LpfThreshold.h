#pragma once
#include <Eigen/Dense>

class LpfThreshold {
public:
    LpfThreshold(void);

    void setValues(double alpha, double beta, int jointNumber);
    double adaptiveThreshold(double newSignal, bool high);
    Eigen::VectorXd adaptiveThreshold(const Eigen::VectorXd& newSignal, bool high);
    double alpha_;
    double beta_;
    
private:
    int jointNumber_;
    double filtered_signal_b_;
    Eigen::VectorXd filtered_signal_;
};