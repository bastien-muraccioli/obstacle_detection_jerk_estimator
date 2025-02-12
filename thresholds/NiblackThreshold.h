#pragma once

#include <Eigen/Dense>
#include <deque>

class NiblackThreshold {
    
public:
    NiblackThreshold(void);

    void setValues(int n, double k, int jointNumber);
    int windowSize;

    double sensitivityThreshold;

    std::deque<Eigen::VectorXd> window;
    Eigen::VectorXd adaptiveThreshold(const Eigen::VectorXd& prevThreshold, const Eigen::VectorXd& newSignal, bool high);

    double prevThreshold_d;
    std::deque<double> window_d;
    double adaptiveThreshold(double prevThreshold, double newSignal, bool high);


private:
    double computeMean_d(void);       // Compute the mean of the sliding window
    double computeStdDev_d(double mean);     // Compute the standard deviation of the sliding window
    Eigen::VectorXd computeMean(void); 
    Eigen::VectorXd computeStdDev(Eigen::VectorXd mean);
    int jointNumber_;
};