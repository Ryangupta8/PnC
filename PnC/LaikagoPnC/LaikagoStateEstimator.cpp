#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/LaikagoPnC/LaikagoDefinition.hpp>
#include <PnC/LaikagoPnC/LaikagoInterface.hpp>
#include <PnC/LaikagoPnC/LaikagoStateEstimator.hpp>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>

LaikagoStateEstimator::LaikagoStateEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(1, "Laikago State Estimator");

    robot_ = robot;
    sp_ = LaikagoStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(Laikago::n_dof);
    curr_qdot_ = Eigen::VectorXd::Zero(Laikago::n_dof);
}

LaikagoStateEstimator::~LaikagoStateEstimator() {}

void LaikagoStateEstimator::Initialization(LaikagoSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    sp_->jpos_ini = curr_config_.segment(Laikago::n_vdof, Laikago::n_adof);
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void LaikagoStateEstimator::Update(LaikagoSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void LaikagoStateEstimator::_JointUpdate(LaikagoSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();
    for (int i = 0; i < Laikago::n_vdof; ++i) {
        curr_config_[i] = data->virtual_q[i];
        curr_qdot_[i] = data->virtual_qdot[i];
    }
    for (int i(0); i < Laikago::n_adof; ++i) {
        curr_config_[Laikago::n_vdof + i] = data->q[i];
        curr_qdot_[Laikago::n_vdof + i] = data->qdot[i];
    }
    // ?? Where do these values come from?
    sp_->front_l_rf = data->front_lf_wrench;
    sp_->front_r_rf = data->front_rf_wrench;
    sp_->rear_l_rf = data->rear_lf_wrench;
    sp_->rear_r_rf = data->rear_rf_wrench;
}

void LaikagoStateEstimator::_ConfigurationAndModelUpdate() {
    robot_->updateSystem(curr_config_, curr_qdot_, true);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
}

void LaikagoStateEstimator::_FootContactUpdate(LaikagoSensorData* data) {
    // Front Contacts
    if (data->front_rfoot_contact)
        sp_->b_front_rfoot_contact = 1;
    else
        sp_->b_front_rfoot_contact = 0;
    if (data->front_lfoot_contact)
        sp_->b_front_lfoot_contact = 1;
    else
        sp_->b_front_lfoot_contact = 0;
    // Rear Contacts
    if (data->rear_rfoot_contact)
        sp_->b_rear_rfoot_contact = 1;
    else
        sp_->b_rear_rfoot_contact = 0;
    if (data->rear_lfoot_contact)
        sp_->b_rear_lfoot_contact = 1;
    else
        sp_->b_rear_lfoot_contact = 0;
}
