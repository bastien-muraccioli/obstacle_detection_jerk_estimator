#include "NiblackThreshold.h"

NiblackThreshold::NiblackThreshold(void){}

void NiblackThreshold::setValues(int n, double k, int jointNumber) {
    windowSize = n;
    sensitivityThreshold = k;
    jointNumber_ = jointNumber;
}

double NiblackThreshold::adaptiveThreshold(double prevThreshold, double newSignal, bool high) {
    window_d.push_back(newSignal);
    if (window_d.size() > windowSize) {
        window_d.pop_front();
    }
    double mean = computeMean_d();
    double stdDev = computeStdDev_d(mean);
    if (high) {
        prevThreshold = mean + sensitivityThreshold * stdDev;
    } else {
        prevThreshold = mean - sensitivityThreshold * stdDev;
    }
    return prevThreshold;
}

Eigen::VectorXd NiblackThreshold::adaptiveThreshold(const Eigen::VectorXd& prevThreshold, const Eigen::VectorXd& newSignal, bool high) {
    window.push_back(newSignal);
    if (window.size() > windowSize) {
        window.pop_front();
    }
    Eigen::VectorXd mean = computeMean();
    Eigen::VectorXd stdDev = computeStdDev(mean);
    Eigen::VectorXd newThreshold = Eigen::VectorXd::Zero(jointNumber_);
    if (high) {
        newThreshold = mean + sensitivityThreshold * stdDev;
    } else {
        newThreshold = mean - sensitivityThreshold * stdDev;
    }
    return newThreshold;
}


double NiblackThreshold::computeMean_d(void) {
    if (window_d.empty()) return 0.0;
    double sum = 0.0;
    for (double val : window_d) {
        sum += val;
    }
    return sum / window_d.size();
}

double NiblackThreshold::computeStdDev_d(double mean) {
    if (window_d.size() < 2) return 0.0;
    double sumSq = 0.0;
    for (double val : window_d) {
        sumSq += (val - mean) * (val - mean);
    }
    return std::sqrt(sumSq / (window_d.size() - 1));
}

Eigen::VectorXd NiblackThreshold::computeMean(void) {
    if (window.empty()) return Eigen::VectorXd::Zero(jointNumber_);
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(jointNumber_);
    for (Eigen::VectorXd val : window) {
        sum += val;
    }
    return sum / window.size();
}

Eigen::VectorXd NiblackThreshold::computeStdDev(Eigen::VectorXd mean) {
    if (window.size() < 2) return Eigen::VectorXd::Zero(jointNumber_);
    Eigen::VectorXd sumSq = Eigen::VectorXd::Zero(jointNumber_);
    for (Eigen::VectorXd val : window) {
        sumSq += (val - mean).cwiseProduct(val - mean);
    }
    return (sumSq / (window.size() - 1)).cwiseSqrt();
}

void NiblackThreshold::reset(void) {
    window.clear();
    window_d.clear();
}