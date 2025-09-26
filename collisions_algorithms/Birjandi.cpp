#include "Birjandi.h"
#include <mc_rtc/logging.h>
#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <unsupported/Eigen/MatrixFunctions> 


constexpr unsigned Birjandi::STATE_SIZE;
constexpr unsigned Birjandi::MEASUREMENT_SIZE;
constexpr unsigned Birjandi::INPUT_SIZE;

Birjandi::Birjandi(void) : filter_(STATE_SIZE, MEASUREMENT_SIZE, INPUT_SIZE, false),  stateDynamics_(0.001)
{
}

void Birjandi::init(mc_control::MCGlobalController & ctl, std::string imuBodyName, std::string jointBodyName, std::string robotBodyName)
{
    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot();
    kt = {
        {"joint_1", 0.11}, 
        {"joint_2", 0.11}, 
        {"joint_3", 0.11},
        {"joint_4", 0.11}, 
        {"joint_5", 0.076}, 
        {"joint_6", 0.076},
        {"joint_7", 0.076}
      };
    // Copy the realrobot to the controller
    robot_copied_ = mc_rbdyn::Robots::make();
    robot_copied_->robotCopy(robot, robot.name());
    robot_copied_->robotCopy(realRobot, "inputRobot");

    auto & inputRobot = robot_copied_->robot("inputRobot");
    forwardDynamics_hat_ = rbd::ForwardDynamics(inputRobot.mb());
    forwardDynamics_ = rbd::ForwardDynamics(realRobot.mb());

    jointNumber_ = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
    imuBodyName_ = imuBodyName;
    robotBodyName_ = robotBodyName;
    
    imu_not_yet_initialized_ = true;
    dt_ = ctl.timestep();

    stateDynamics_.setSamplingPeriod(dt_);
    filter_.setFunctor(&stateDynamics_);
    jointBodyName_ = jointBodyName;

    R_joint_ = realRobot.bodyPosW(jointBodyName_).rotation();

    accelero_ = Eigen::Vector3d::Zero();
    ya_ = Eigen::Vector3d::Zero();
    gyro_ = Eigen::Vector3d::Zero();
    angveldot_ = Eigen::Vector3d::Zero();
    acclin_joint_ = Eigen::Vector3d::Zero();
    angvel_joint_ = Eigen::Vector3d::Zero();

    q_ = realRobot.encoderValues()[jointNumber_-2]; // Initial joint position from encoders
    mc_rtc::log::info("[Birjandi] Initial q where IMU is set: {} rad", q_);
    qdot_hat_ = 0.0;
    qddot_hat_ = 0.0;

    qdot_ = 0.0;
    qddot_ = 0.0;
    qddot_qp_ = 0.0;
    qddot_accelero_ = 0.0;

    // Compute the covariance matrix
    Eigen::VectorXd v(7); // Dimension of q + accelero + gyro
    v << qCovariance_, acceleroCovariance_, acceleroCovariance_, acceleroCovariance_, gyroCovariance_, gyroCovariance_, gyroCovariance_;
    r_covariance_ = v.asDiagonal();
    q_covariance_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * stateCov;
    mc_rtc::log::info("[Birjandi] R size: {}x{}", r_covariance_.rows(), r_covariance_.cols());
    mc_rtc::log::info("[Birjandi] Q size: {}x{}", q_covariance_.rows(), q_covariance_.cols());

    // Initialize EKF covariances
    filter_.setQ(q_covariance_);
    filter_.setR(r_covariance_);
    filter_.setStateCovariance(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * stateInitCov);
    

    // Define system matrix A (continuous-time state transition model)
   
    // mc_rtc::log::info("[Birjandi] A size: {}x{}", A.rows(), A.cols());
    // mc_rtc::log::info("[Birjandi] C size: {}x{}", C.rows(), C.cols());

    // auto time = filter_.getCurrentTime();
    
    Eigen::VectorXd x0(STATE_SIZE);
    x0 << q_, 0, 0, 0;
    filter_.setState(x0, 0);

    // Eigen::VectorXd u0(INPUT_SIZE);
    // u0 << 0, 0, 0,  // Acceleration
    //     0, 0, 0,  // Angular velocity
    //     0, 0, 0,  // Previous angular velocity
    //     0, 0, 0, // Angular acceleration
    //     0, 0, 0;  // Previous angular acceleration
    // filter_.setInput(u0, 0);

    Eigen::VectorXd y0(MEASUREMENT_SIZE);
    y0 << q_, 0, 0, 0, 0, 0, 0;
    filter_.setMeasurement(y0, 0);
    tau_m = Eigen::VectorXd::Zero(jointNumber_);
    tau_fric = Eigen::VectorXd::Zero(jointNumber_);
    // filter_.setInput(y0, 0);


    mc_rtc::log::info("[Birjandi] Initialized Birjandi filter");
}

void Birjandi::computeTauExtRealRobot(mc_control::MCGlobalController & ctl)
{
    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot();

    realRobot.forwardKinematics();
    realRobot.forwardVelocity();
    realRobot.forwardAcceleration();

    forwardDynamics_.computeC(realRobot.mb(), realRobot.mbc());
    forwardDynamics_.computeH(realRobot.mb(), realRobot.mbc());

    auto coriolis = new rbd::Coriolis(robot.mb());
    coriolisMatrix_ = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
    auto forwardDynamics = rbd::ForwardDynamics(robot.mb());

    forwardDynamics.computeH(robot.mb(), robot.mbc());
    inertiaMatrix_ = forwardDynamics.H() - forwardDynamics.HIr();

    if(ctl.controller().datastore().has("torque_fric"))
    {
      tau_fric = ctl.controller().datastore().get<Eigen::VectorXd>("torque_fric");
    }
    else
    {
      tau_fric.setZero(jointNumber_);
      mc_rtc::log::error("[Birjandi] No torque_fric in datastore");
    }
    int jointIndex = 0;
    for(auto const& [key, val] : kt)
    {
        if (jointIndex >= 0 && jointIndex < jointNumber_) {
            // Calculate motor torque and assign it to tau_m
            double tau_mot = val * gear_ratio * realRobot.jointJointSensor(key).motorCurrent();
            tau_m[jointIndex] = tau_mot;
        } else {
            mc_rtc::log::error("[Birjandi] Invalid joint name: {} or index out of bounds", key);
        }
        jointIndex++;
    }

    auto coriolisGravityTerm = forwardDynamics_.C(); //C*qdot + g
    tau_ext_ = inertiaMatrix_ * qddot_hat_ + coriolisGravityTerm + tau_fric - tau_m;
}

void Birjandi::computeTauExtInputRobot(mc_control::MCGlobalController & ctl)
{
    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot();
    auto & inputRobot = robot_copied_->robot("inputRobot");

    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();

    inputRobot.forwardKinematics();
    inputRobot.forwardVelocity();
    inputRobot.forwardAcceleration();


    auto & realQ = robot.mbc().q;
    auto & realAlpha = robot.mbc().alpha;
    auto & realAlphaD = robot.mbc().alphaD;

    Eigen::VectorXd realQ_v(jointNumber_);
    Eigen::VectorXd realAlpha_v(jointNumber_);
    Eigen::VectorXd realAlphaD_v(jointNumber_);

    rbd::paramToVector(robot.mbc().q, realQ_v);
    rbd::paramToVector(robot.mbc().alpha, realAlpha_v);
    rbd::paramToVector(robot.mbc().alphaD, realAlphaD_v);

    // mc_rtc::log::info("[Birjandi] Real qdot: {}, Real qddot: {}", realAlpha_v[5], realAlphaD_v[5]);
    // mc_rtc::log::info("[Birjandi] Esti qdot: {}, Esti qddot: {}", qdot_hat_, qddot_hat_);

    Eigen::VectorXd realQ_v_hat(jointNumber_);
    Eigen::VectorXd realAlpha_v_hat(jointNumber_);
    Eigen::VectorXd realAlphaD_v_hat(jointNumber_);

    qddot_qp_ = realAlphaD_v[5];

    realQ_v_hat = realQ_v;
    realAlpha_v_hat = realAlpha_v;
    realAlphaD_v_hat = realAlphaD_v;
    realAlpha_v_hat[5] = qdot_hat_;
    realAlphaD_v_hat[5] = qddot_hat_;


    rbd::vectorToParam(realQ_v_hat, realQ);
    rbd::vectorToParam(realAlpha_v_hat, realAlpha);
    rbd::vectorToParam(realAlphaD_v_hat, realAlphaD);


    std::copy(std::next(realQ.begin()), realQ.end(), std::next(inputRobot.mbc().q.begin()));
    std::copy(std::next(realAlpha.begin()), realAlpha.end(), std::next(inputRobot.mbc().alpha.begin()));
    std::copy(std::next(realAlphaD.begin()), realAlphaD.end(), std::next(inputRobot.mbc().alphaD.begin()));


    Eigen::VectorXd realQ_v_hat_temp(jointNumber_);
    Eigen::VectorXd realAlpha_v_hat_temp(jointNumber_);
    Eigen::VectorXd realAlphaD_v_hat_temp(jointNumber_);

    
    rbd::paramToVector(inputRobot.mbc().q, realQ_v_hat_temp);
    rbd::paramToVector(inputRobot.mbc().alpha, realAlpha_v_hat_temp);
    rbd::paramToVector(inputRobot.mbc().alphaD, realAlphaD_v_hat_temp);

    // mc_rtc::log::info("[Birjandi] Real q: {}, Real qdot: {}, Real qddot: {}",realQ_v[5], realAlpha_v[5], realAlphaD_v[5]);
    // mc_rtc::log::info("[Birjandi] mbc  q: {}, mbc  qdot: {}, mbc  qddot: {}", realQ_v_hat[5], realAlpha_v_hat[5], realAlphaD_v_hat[5]);
    // mc_rtc::log::info("[Birjandi] mbcb q: {}, mbcb qdot: {}, mbcb qddot: {}", realQ_v_hat_temp[5], realAlpha_v_hat_temp[5], realAlphaD_v_hat_temp[5]);

    forwardDynamics_hat_.computeC(inputRobot.mb(), inputRobot.mbc());
    forwardDynamics_hat_.computeH(inputRobot.mb(), inputRobot.mbc());

    auto coriolis_hat = new rbd::Coriolis(inputRobot.mb());
    coriolisMatrix_hat_ = coriolis_hat->coriolis(inputRobot.mb(), inputRobot.mbc());
    

    forwardDynamics_hat_.computeH(inputRobot.mb(), inputRobot.mbc());
    inertiaMatrix_hat_ = forwardDynamics_hat_.H() - forwardDynamics_hat_.HIr();

    if(ctl.controller().datastore().has("torque_fric"))
    {
      tau_fric = ctl.controller().datastore().get<Eigen::VectorXd>("torque_fric");
    }
    else
    {
      tau_fric.setZero(jointNumber_);
      mc_rtc::log::error("[Birjandi] No torque_fric in datastore");
    }
    int jointIndex = 0;
    for(auto const& [key, val] : kt)
    {
        if (jointIndex >= 0 && jointIndex < jointNumber_) {
            // Calculate motor torque and assign it to tau_m
            double tau_mot = val * gear_ratio * realRobot.jointJointSensor(key).motorCurrent();
            tau_m[jointIndex] = tau_mot;
        } else {
            mc_rtc::log::error("[Birjandi] Invalid joint name: {} or index out of bounds", key);
        }
        jointIndex++;
    }

    auto coriolisGravityTerm = forwardDynamics_hat_.C(); //C*qdot + g
    tau_ext_hat_ = inertiaMatrix_hat_ * qddot_hat_ + coriolisGravityTerm + tau_fric - tau_m;
}

void Birjandi::collisionDetection(mc_control::MCGlobalController & ctl)
{
    updateFilter(ctl);
    computeTauExtRealRobot(ctl);
    computeTauExtInputRobot(ctl);
    // mc_rtc::log::info("[Birjandi] Inertia matrix hat: {}, Inertia Matrix real: {}", inertiaMatrix_hat_.norm(), inertiaMatrix_.norm());
    // mc_rtc::log::info("[Birjandi] Coriolis hat: {}, Coriolis real: {}", coriolisMatrix_hat_.norm(), coriolisMatrix_.norm());
    
}

void Birjandi::updateFilter(mc_control::MCGlobalController & ctl)
{
    auto & robot = ctl.robot();
    auto & realRobot = ctl.realRobot();

    if(!robot.hasBodySensor(imuBodyName_))
    {
        mc_rtc::log::error("[Birjandi] Body sensor {} does not exist in the robot. Jerk Estimation is impossible", imuBodyName_);
        return;
    }

    const mc_rbdyn::BodySensor & imu = robot.bodySensor(imuBodyName_);

    if(imu.linearAcceleration().norm() != 0.0 && imu.angularVelocity().norm() != 0.0 && imu_not_yet_initialized_)
    {
        imu_not_yet_initialized_ = false;
        accelero_ = imu.linearAcceleration();
        gyro_ = imu.angularVelocity();
    }

    if(imu_not_yet_initialized_)
    {
        return;
    }

    
    // Measurements
    q_ = realRobot.encoderValues()[jointNumber_-2];
    // auto R_imu_w = realRobot.bodySensor(imuBodyName_).orientation().toRotationMatrix();
    auto R_imu_w = realRobot.bodyPosW(robotBodyName_).rotation();
    auto R_joint_w = realRobot.bodyPosW(jointBodyName_).rotation();
    R_joint_ = R_joint_w * R_imu_w.transpose();
    stateDynamics_.setJointOrientation(R_joint_);
    

    // Computing the acceleration and angular velocity in the joint frame
    Eigen::VectorXd prev_angvel_ = angvel_joint_;
    
    auto P_imu = realRobot.bodySensor(imuBodyName_).position();
    auto P_joint = realRobot.bodyPosW(jointBodyName_).translation();
    auto X_pos = R_joint_.transpose() * (P_imu - P_joint);
    stateDynamics_.setJointPosition(X_pos);

    gyro_ = R_joint_.transpose() * imu.angularVelocity();
    gyro_[1] = -gyro_[1];
    accelero_ = R_joint_.transpose() * imu.linearAcceleration();
    // accelero_[1] = -accelero_[1];
    
    Eigen::VectorXd y(7);
    y << q_, accelero_.x(), accelero_.y(), accelero_.z(), gyro_.x(), gyro_.y(), gyro_.z();
    // mc_rtc::log::info("[Birjandi] Assembled measurement vector: [{}, {}, {}, {}, {}, {}, {}]", y(0), y(1), y(2), y(3), y(4), y(5), y(6));

    Eigen::MatrixXd A(4, 4);
    A <<  0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
        0, 0, 0, 0;
    // Discretize A
    Eigen::MatrixXd A_discrete(4, 4);
    A_discrete = (A * dt_).exp();
    filter_.setA(A_discrete);
    // mc_rtc::log::info("[Birjandi] Defined system matrix A");

    
    const stateObservation::Vector dx = filter_.stateVectorConstant(1) * 1e-8;
    auto CFD = filter_.getCMatrixFD(dx);
    // mc_rtc::log::info("[Birjandi] C size: {}x{}", CFD.rows(), CFD.cols());
    filter_.setC(CFD);

    // mc_rtc::log::info("[Birjandi] Defined measurement matrix C");
    
    auto time = filter_.getCurrentTime();
    filter_.setMeasurement(y, time + 1);
    
    // Run EKF and update state estimate
    auto xk = filter_.getEstimatedState(time + 1);
    // mc_rtc::log::info("[Birjandi] Got estimated state {}", xk);
    qdot_hat_ = xk(1);
    qddot_hat_ = xk(2);
    ya_ = stateDynamics_.getYaMeasurement();
    // mc_rtc::log::info("[Birjandi] ya: {}", ya_);
    // mc_rtc::log::info("[Birjandi] Estimated qdot: {}, Estimated qddot: {}", qdot_hat_, qddot_hat_);


    // Ground truth (derived from encoders)
    double qdot_prev = qdot_;
    qdot_ = realRobot.encoderVelocities()[jointNumber_-2];
    double qddot_prev = qddot_;
    qddot_ = (qdot_ - qdot_prev) / dt_;
    // filtered
    qddot_ = a_filter_ * qddot_ + (1-a_filter_) * qddot_prev;
    qddot_accelero_ = (imu.linearAcceleration()[1]-9.81)/distance_imu_joint_;

    // mc_rtc::log::info("[Birjandi] Ground truth qdot: {}, Ground truth qddot: {}", qdot_, qddot_);
}

void Birjandi::addPlot(std::vector<std::string> & plots, double counter, mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    counter_ = counter;
    // gui.addPlot(
    //     "q Birjandi",
    //     mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
    //     mc_rtc::gui::plot::Y("q", [this]() { return q_; }, mc_rtc::gui::Color::Green)
    // );
    gui.addPlot(
        "qdot Birjandi",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("gyro", [this]() { return gyro_[1]; }, mc_rtc::gui::Color::Blue),
        mc_rtc::gui::plot::Y("qdot_hat", [this]() { return qdot_hat_; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y("qdot", [this]() { return qdot_; }, mc_rtc::gui::Color::Green)
    );
    gui.addPlot(
        "qddot Birjandi",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("qddot_accelero", [this]() { return qddot_accelero_; }, mc_rtc::gui::Color::Cyan),
        mc_rtc::gui::plot::Y("qddot_qp", [this]() { return qddot_qp_; }, mc_rtc::gui::Color::Magenta),
        mc_rtc::gui::plot::Y("ya", [this]() { return ya_[1]; }, mc_rtc::gui::Color::Blue),
        mc_rtc::gui::plot::Y("qddot_hat", [this]() { return qddot_hat_; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y("qddot", [this]() { return qddot_; }, mc_rtc::gui::Color::Green)
    );

    gui.addPlot(
        "tau_ext Birjandi",
        mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y("tau_ext_hat", [this]() { return tau_ext_hat_[5]; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y("tau_ext", [this]() { return tau_ext_[5]; }, mc_rtc::gui::Color::Green)
    );
}

void Birjandi::updateCounter(double counter)
{
    counter_ = counter;
}

void Birjandi::addLog(mc_control::MCGlobalController & ctl)
{
    ctl.controller().logger().addLogEntry("Birjandi_q", [this]() { return q_; });
    ctl.controller().logger().addLogEntry("Birjandi_qdot_hat", [this]() { return qdot_hat_; });
    ctl.controller().logger().addLogEntry("Birjandi_qdot", [this]() { return qdot_; });
    ctl.controller().logger().addLogEntry("Birjandi_qddot_hat", [this]() { return qddot_hat_; });
    ctl.controller().logger().addLogEntry("Birjandi_qddot", [this]() { return qddot_; });
    ctl.controller().logger().addLogEntry("Birjandi_acclin_joint", [this]() { return acclin_joint_; });
    ctl.controller().logger().addLogEntry("Birjandi_angvel_joint", [this]() { return angvel_joint_; });
    ctl.controller().logger().addLogEntry("Birjandi_gyro", [this]() { return gyro_; });
    ctl.controller().logger().addLogEntry("Birjandi_accelero", [this]() { return accelero_; });
    ctl.controller().logger().addLogEntry("Birjandi_angveldot", [this]() { return angveldot_; });
    ctl.controller().logger().addLogEntry("Birjandi_tau_ext_hat", [this]() { return tau_ext_hat_; });
    ctl.controller().logger().addLogEntry("Birjandi_tau_ext", [this]() { return tau_ext_; });
    ctl.controller().logger().addLogEntry("Birjandi_ya", [this]() { return ya_; });
}
