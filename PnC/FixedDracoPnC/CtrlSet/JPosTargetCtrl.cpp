#include <PnC/FixedDracoPnC/CtrlSet/JPosTargetCtrl.hpp>
#include <PnC/FixedDracoPnC/FixedDracoDefinition.hpp>
#include <PnC/FixedDracoPnC/FixedDracoInterface.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

JPosTargetCtrl::JPosTargetCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "JPos Target Ctrl");
    ctrl_count_ = 0;
    ctrl_time_ = 0.;
}

JPosTargetCtrl::~JPosTargetCtrl() {}

void JPosTargetCtrl::oneStep(void* _cmd) {
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(robot_->getNumDofs());
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(robot_->getNumDofs());
    double q_des, qdot_des, qddot_ff;
    Eigen::VectorXd q = robot_->getQ();
    Eigen::VectorXd qdot = robot_->getQdot();

    for (int i = 0; i < robot_->getNumDofs(); ++i) {
        q_des = myUtils::smooth_changing(ini_pos_[i], target_pos_[i],
                                         move_time_, ctrl_time_);
        qdot_des = myUtils::smooth_changing_vel(ini_pos_[i], target_pos_[i],
                                                move_time_, ctrl_time_);
        qddot_ff = myUtils::smooth_changing_acc(ini_pos_[i], target_pos_[i],
                                                move_time_, ctrl_time_);
        qddot[i] =
            qddot_ff + kp_[i] * (q_des - q[i]) + kd_[i] * (qdot_des - qdot[i]);
    }
    gamma = robot_->getMassMatrix() * qddot + robot_->getCoriolisGravity();

    ((FixedDracoCommand*)_cmd)->jtrq = gamma;

    ++ctrl_count_;
    ctrl_time_ = ctrl_count_ * FixedDracoAux::ServoRate;
}

void JPosTargetCtrl::firstVisit() {
    ctrl_time_ = 0.;
    ctrl_count_ = 0;
    ini_pos_ = robot_->getQ();
    ini_vel_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
}

void JPosTargetCtrl::lastVisit() {}

bool JPosTargetCtrl::endOfPhase() {
    if (ctrl_time_ > end_time_) {
        return true;
    }
    return false;
}

void JPosTargetCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
        myUtils::readParameter(node, "kp", kp_);
        myUtils::readParameter(node, "kd", kd_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
