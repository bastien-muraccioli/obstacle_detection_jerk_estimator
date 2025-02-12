#pragma once

#include <mc_control/GlobalPlugin.h>
#include "../thresholds/NiblackThreshold.h"
#include "../thresholds/CusumThreshold.h"
#include "../thresholds/LpfThreshold.h"

class Zurlo {
    
public:
    Zurlo(void);

    void setValues(int n, int jointNumber, double kNiblack=8.0, double kCusum=0.5, bool useNiblackThreshold=true, bool useCusumThreshold=false);
    bool collisionDetection(mc_control::MCGlobalController & ctl);
    void addPlot(std::vector<std::string> & plots, double counter, mc_control::MCGlobalController & ctl);
    void addGui(mc_control::MCGlobalController & ctl);
    void addLog(mc_control::MCGlobalController & ctl);
    void updateCounter(double counter);
    void resetThresholds(void);

    double base_high_threshold;
    double base_low_threshold;
    Eigen::VectorXd residual_high_threshold;
    Eigen::VectorXd residual_low_threshold;
    Eigen::VectorXd residual_current_high_threshold;
    Eigen::VectorXd residual_current_low_threshold;
    double residual_energy_high_threshold;
    double residual_energy_low_threshold;
    bool zurloEstimationFlag_;
    bool zurloUse_residual_;
    bool zurloUse_residual_current_;
    Eigen::VectorXd residual_;
    Eigen::VectorXd residual_current_;
    double residual_energy_;
    bool zurloEstimationControlFlag_;
    int windowSize;             
    double sensitivityThresholdNiblack;
    double sensitivityThresholdCusum; 

    NiblackThreshold zurloNiblackThreshold_residual_;
    NiblackThreshold zurloNiblackThreshold_residual_current_;
    NiblackThreshold zurloNiblackThreshold_residual_energy_;

    CusumThreshold zurloCusumThreshold_residual_;
    CusumThreshold zurloCusumThreshold_residual_current_;
    CusumThreshold zurloCusumThreshold_residual_energy_;

    double alpha_;
    double beta_;

    LpfThreshold zurloLpfThreshold_residual_;
    LpfThreshold zurloLpfThreshold_residual_current_;
    LpfThreshold zurloLpfThreshold_residual_energy_;

    // Detection observer
    bool detection_zurlo_current_;
    bool detection_zurlo_torque_;

    bool useNiblackThreshold_;
    bool useCusumThreshold_;
    bool useLpfThreshold_;


private:
    double counter_;
    int jointNumber_;

};