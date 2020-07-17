#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/LaikagoPnC/LaikagoDefinition.hpp>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

LaikagoStateProvider* LaikagoStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static LaikagoStateProvider state_provider_(_robot);
    return &state_provider_;
}

LaikagoStateProvider::LaikagoStateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "Laikago State Provider");

    // API related parameters
    b_walking = false;
    ft_length = 0.;
    r_ft_width = 0.;
    l_ft_width = 0.;
    ft_ori_inc = 0.;
    num_total_step = 0;
    num_residual_step = 0;

    num_step_copy = 0;
    phase_copy = 0;
    robot_ = _robot;
    stance_foot_1 = LaikagoBodyNode::FL_foot;
    stance_foot_2 = LaikagoBodyNode::RR_foot;
    curr_time = 0.;
    prev_state_machine_time = 0.;
    planning_moment = 0.;

    q = Eigen::VectorXd::Zero(Laikago::n_dof);
    qdot = Eigen::VectorXd::Zero(Laikago::n_dof);

    b_front_rfoot_contact = 0;
    b_front_lfoot_contact = 0;
    b_rear_rfoot_contact = 0;
    b_rear_lfoot_contact = 0;

    foot_target_list.clear();
    com_des_list.clear();

    com_pos = Eigen::VectorXd::Zero(3);
    com_vel = Eigen::VectorXd::Zero(3);
    mom = Eigen::VectorXd::Zero(6);

    com_pos_des = Eigen::VectorXd::Zero(3);
    com_vel_des = Eigen::VectorXd::Zero(3);
    mom_des = Eigen::VectorXd::Zero(6);
    // Actual foot velocity and position
    front_rf_pos = Eigen::VectorXd::Zero(3);
    front_rf_vel = Eigen::VectorXd::Zero(3);
    front_lf_pos = Eigen::VectorXd::Zero(3);
    front_lf_vel = Eigen::VectorXd::Zero(3);
    rear_rf_pos = Eigen::VectorXd::Zero(3);
    rear_rf_vel = Eigen::VectorXd::Zero(3);
    rear_lf_pos = Eigen::VectorXd::Zero(3);
    rear_lf_vel = Eigen::VectorXd::Zero(3);
    // Desired foot velocity and position
    front_rf_pos_des = Eigen::VectorXd::Zero(3);
    front_rf_vel_des = Eigen::VectorXd::Zero(3);
    front_lf_pos_des = Eigen::VectorXd::Zero(3);
    front_lf_vel_des = Eigen::VectorXd::Zero(3);
    rear_rf_pos_des = Eigen::VectorXd::Zero(3);
    rear_rf_vel_des = Eigen::VectorXd::Zero(3);
    rear_lf_pos_des = Eigen::VectorXd::Zero(3);
    rear_lf_vel_des = Eigen::VectorXd::Zero(3);
    // Actual foot angular velocity and orientation
    front_rf_ori_quat = Eigen::Quaternion<double>::Identity();
    front_rf_ang_vel = Eigen::VectorXd::Zero(3);
    front_lf_ori_quat = Eigen::Quaternion<double>::Identity();
    front_lf_ang_vel = Eigen::VectorXd::Zero(3);
    rear_rf_ori_quat = Eigen::Quaternion<double>::Identity();
    rear_rf_ang_vel = Eigen::VectorXd::Zero(3);
    rear_lf_ori_quat = Eigen::Quaternion<double>::Identity();
    rear_lf_ang_vel = Eigen::VectorXd::Zero(3);
    // Desired foot angular velocity and orientation
    front_rf_ori_quat_des = Eigen::Quaternion<double>::Identity();
    front_rf_ang_vel_des = Eigen::VectorXd::Zero(3);
    front_lf_ori_quat_des = Eigen::Quaternion<double>::Identity();
    front_lf_ang_vel_des = Eigen::VectorXd::Zero(3);
    rear_rf_ori_quat_des = Eigen::Quaternion<double>::Identity();
    rear_rf_ang_vel_des = Eigen::VectorXd::Zero(3);
    rear_lf_ori_quat_des = Eigen::Quaternion<double>::Identity();
    rear_lf_ang_vel_des = Eigen::VectorXd::Zero(3);

    trunk_ori = Eigen::Quaternion<double>::Identity();
    trunk_ang_vel = Eigen::VectorXd::Zero(3);

    trunk_ori_des = Eigen::Quaternion<double>::Identity();
    trunk_ang_vel_des = Eigen::VectorXd::Zero(3);

    // ??
    // r_rf = Eigen::VectorXd::Zero(6);
    // l_rf = Eigen::VectorXd::Zero(6);
    // r_rf_des = Eigen::VectorXd::Zero(6);
    // l_rf_des = Eigen::VectorXd::Zero(6);

    des_jacc_cmd = Eigen::VectorXd::Zero(Laikago::n_adof);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&curr_time, DOUBLE, "time");
    data_manager->RegisterData(&q, VECT, "q", Laikago::n_dof);
    data_manager->RegisterData(&qdot, VECT, "qdot", Laikago::n_dof);
    data_manager->RegisterData(&b_front_rfoot_contact, INT, "front_rfoot_contact", 1);
    data_manager->RegisterData(&b_front_lfoot_contact, INT, "front_lfoot_contact", 1);
    data_manager->RegisterData(&b_rear_rfoot_contact, INT, "rear_rfoot_contact", 1);
    data_manager->RegisterData(&b_rear_lfoot_contact, INT, "rear_lfoot_contact", 1);

    data_manager->RegisterData(&com_pos, VECT, "com_pos", 3);
    data_manager->RegisterData(&com_vel, VECT, "com_vel", 3);
    data_manager->RegisterData(&mom, VECT, "cm", 6);

    data_manager->RegisterData(&com_pos_des, VECT, "com_pos_des", 3);
    data_manager->RegisterData(&com_vel_des, VECT, "com_vel_des", 3);
    data_manager->RegisterData(&mom_des, VECT, "cm_des", 6);

    data_manager->RegisterData(&front_rf_pos, VECT, "front_rf_pos", 3);
    data_manager->RegisterData(&front_rf_vel, VECT, "front_rf_vel", 3);
    data_manager->RegisterData(&front_lf_pos, VECT, "front_lf_pos", 3);
    data_manager->RegisterData(&front_lf_vel, VECT, "front_lf_vel", 3);
    data_manager->RegisterData(&rear_rf_pos, VECT, "rear_rf_pos", 3);
    data_manager->RegisterData(&rear_rf_vel, VECT, "rear_rf_vel", 3);
    data_manager->RegisterData(&rear_lf_pos, VECT, "rear_lf_pos", 3);
    data_manager->RegisterData(&rear_lf_vel, VECT, "rear_lf_vel", 3);

    data_manager->RegisterData(&front_rf_pos_des, VECT, "front_rf_pos_des", 3);
    data_manager->RegisterData(&front_rf_vel_des, VECT, "front_rf_vel_des", 3);
    data_manager->RegisterData(&front_lf_pos_des, VECT, "front_lf_pos_des", 3);
    data_manager->RegisterData(&front_lf_vel_des, VECT, "front_lf_vel_des", 3);
    data_manager->RegisterData(&rear_rf_pos_des, VECT, "rear_rf_pos_des", 3);
    data_manager->RegisterData(&rear_rf_vel_des, VECT, "rear_rf_vel_des", 3);
    data_manager->RegisterData(&rear_lf_pos_des, VECT, "rear_lf_pos_des", 3);
    data_manager->RegisterData(&rear_lf_vel_des, VECT, "rear_lf_vel_des", 3);

    data_manager->RegisterData(&front_rf_ori_quat, QUATERNION, "front_rf_ori_quat", 4);
    data_manager->RegisterData(&front_rf_ang_vel, VECT, "front_rf_ang_vel", 3);
    data_manager->RegisterData(&front_lf_ori_quat, QUATERNION, "front_lf_ori_quat", 4);
    data_manager->RegisterData(&front_lf_ang_vel, VECT, "front_lf_ang_vel", 3);
    data_manager->RegisterData(&rear_rf_ori_quat, QUATERNION, "rear_rf_ori_quat", 4);
    data_manager->RegisterData(&rear_rf_ang_vel, VECT, "rear_rf_ang_vel", 3);
    data_manager->RegisterData(&rear_lf_ori_quat, QUATERNION, "rear_lf_ori_quat", 4);
    data_manager->RegisterData(&rear_lf_ang_vel, VECT, "rear_lf_ang_vel", 3);

    data_manager->RegisterData(&front_rf_ori_quat_des, QUATERNION, "front_rf_ori_quat_des",
                               4);
    data_manager->RegisterData(&front_rf_ang_vel_des, VECT, "front_rf_ang_vel_des", 3);
    data_manager->RegisterData(&front_lf_ori_quat_des, QUATERNION, "front_lf_ori_quat_des",
                               4);
    data_manager->RegisterData(&front_lf_ang_vel_des, VECT, "front_lf_ang_vel_des", 3);
    data_manager->RegisterData(&rear_rf_ori_quat_des, QUATERNION, "rear_rf_ori_quat_des",
                               4);
    data_manager->RegisterData(&rear_rf_ang_vel_des, VECT, "rear_rf_ang_vel_des", 3);
    data_manager->RegisterData(&rear_lf_ori_quat_des, QUATERNION, "rear_lf_ori_quat_des",
                               4);
    data_manager->RegisterData(&rear_lf_ang_vel_des, VECT, "rear_lf_ang_vel_des", 3);

    data_manager->RegisterData(&trunk_ori, QUATERNION, "trunk_ori", 4);
    data_manager->RegisterData(&trunk_ang_vel, VECT, "trunk_ang_vel", 3);

    data_manager->RegisterData(&trunk_ori_des, QUATERNION, "trunk_ori_des",
                               4);
    data_manager->RegisterData(&trunk_ang_vel_des, VECT, "trunk_ang_vel_des",
                               3);

    // ??
    data_manager->RegisterData(&front_r_rf_des, VECT, "front_r_rf_des", 6);
    data_manager->RegisterData(&front_l_rf_des, VECT, "front_l_rf_des", 6);
    data_manager->RegisterData(&front_r_rf, VECT, "front_r_rf", 6);
    data_manager->RegisterData(&front_l_rf, VECT, "front_l_rf", 6);
    data_manager->RegisterData(&rear_r_rf_des, VECT, "rear_r_rf_des", 6);
    data_manager->RegisterData(&rear_l_rf_des, VECT, "rear_l_rf_des", 6);
    data_manager->RegisterData(&rear_r_rf, VECT, "rear_r_rf", 6);
    data_manager->RegisterData(&rear_l_rf, VECT, "rear_l_rf", 6);

    data_manager->RegisterData(&des_jacc_cmd, VECT, "des_jacc_cmd",
                               Laikago::n_adof);
}

void LaikagoStateProvider::saveCurrentData() {
    for (int i = 0; i < 3; ++i) {
        com_pos[i] = robot_->getCoMPosition()[i];
        com_vel[i] = robot_->getCoMVelocity()[i];
    } 
    mom = robot_->getCentroidMomentum();

    front_rf_pos = robot_->getBodyNodeIsometry(LaikagoBodyNode::FR_foot)
                 .translation();
    front_rf_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::FR_foot)
            .tail(3);
    front_lf_pos = robot_->getBodyNodeIsometry(LaikagoBodyNode::FL_foot)
                 .translation();
    front_lf_vel = robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::FL_foot)
                 .tail(3);
    rear_rf_pos = robot_->getBodyNodeIsometry(LaikagoBodyNode::RR_foot)
                 .translation();
    rear_rf_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::RR_foot)
            .tail(3);
    rear_lf_pos = robot_->getBodyNodeIsometry(LaikagoBodyNode::RL_foot)
                 .translation();
    rear_lf_vel = robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::RL_foot)
                 .tail(3);

    front_rf_ori_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(LaikagoBodyNode::FR_foot).linear());
    front_rf_ang_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::FR_foot)
            .head(3);
    front_lf_ori_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(LaikagoBodyNode::FL_foot).linear());
    front_lf_ang_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::FL_foot)
            .head(3);
    rear_rf_ori_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(LaikagoBodyNode::RR_foot).linear());
    rear_rf_ang_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::RR_foot)
            .head(3);
    rear_lf_ori_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(LaikagoBodyNode::RL_foot).linear());
    rear_lf_ang_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::RL_foot)
            .head(3);

    trunk_ori = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(LaikagoBodyNode::trunk).linear());

    trunk_ang_vel =
        robot_->getBodyNodeSpatialVelocity(LaikagoBodyNode::trunk).head(3);
}
