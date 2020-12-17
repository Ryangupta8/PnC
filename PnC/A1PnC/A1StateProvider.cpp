#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

A1StateProvider* A1StateProvider::getStateProvider(RobotSystem* _robot){
    static A1StateProvider state_provider(_robot);
    return &state_provider;
}

A1StateProvider::A1StateProvider(RobotSystem* _robot){
    myUtils::pretty_constructor(1,"A1 State Provider");

    // API related parameters
    b_walking = false;
    b_ready_to_walk = false;
    ft_length = 0.;
    fr_ft_width = 0.;
    fl_ft_width = 0.;
    rl_ft_width = 0.;
    rr_ft_width = 0.;
    ft_ori_inc = 0.;
    num_total_step = 0;
    num_residual_step = 0;

    num_total_step = 0;
    num_step_copy = 0;
    phase_copy = 0;
    robot_ = _robot;
    curr_time = 0.;

    q_ = Eigen::VectorXd::Zero(A1::n_dof);
    qdot_ = Eigen::VectorXd::Zero(A1::n_dof);
    jpos_ini_ = Eigen::VectorXd::Zero(A1::n_adof);

    b_flfoot_contact = 0;
    b_frfoot_contact = 0;
    b_rlfoot_contact = 0;
    b_rrfoot_contact = 0;

    foot_target_list.clear();
    com_des_list.clear();

    com_pos = Eigen::VectorXd::Zero(3);
    com_vel = Eigen::VectorXd::Zero(3);
    // mom = Eigen::VectorXd::Zero(3);

    com_pos_des = Eigen::VectorXd::Zero(3);
    com_vel_des = Eigen::VectorXd::Zero(3);
    // mom_des = Eigen::VectorXd::Zero(3);

    flfoot_pos = Eigen::VectorXd::Zero(3);
    frfoot_pos = Eigen::VectorXd::Zero(3);
    rlfoot_pos = Eigen::VectorXd::Zero(3);
    rrfoot_pos = Eigen::VectorXd::Zero(3);
    flfoot_vel = Eigen::VectorXd::Zero(3);
    frfoot_vel = Eigen::VectorXd::Zero(3);
    rlfoot_vel = Eigen::VectorXd::Zero(3);
    rrfoot_vel = Eigen::VectorXd::Zero(3);

    flfoot_pos_des = Eigen::VectorXd::Zero(3);
    frfoot_pos_des = Eigen::VectorXd::Zero(3);
    rlfoot_pos_des = Eigen::VectorXd::Zero(3);
    rrfoot_pos_des = Eigen::VectorXd::Zero(3);
    flfoot_vel_des = Eigen::VectorXd::Zero(3);
    frfoot_vel_des = Eigen::VectorXd::Zero(3);
    rlfoot_vel_des = Eigen::VectorXd::Zero(3);
    rrfoot_vel_des = Eigen::VectorXd::Zero(3);

    des_jacc_cmd = Eigen::VectorXd::Zero(A1::n_adof);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&curr_time, DOUBLE, "time");
    data_manager->RegisterData(&q_, VECT, "q", A1::n_adof);
    data_manager->RegisterData(&qdot_, VECT, "qdot", A1::n_adof); 
    data_manager->RegisterData(&b_flfoot_contact, INT, "flfoot_contact", 1);
    data_manager->RegisterData(&b_frfoot_contact, INT, "frfoot_contact", 1);
    data_manager->RegisterData(&b_rlfoot_contact, INT, "rlfoot_contact", 1);
    data_manager->RegisterData(&b_rrfoot_contact, INT, "rrfoot_contact", 1);

    data_manager->RegisterData(&com_pos, VECT, "com_pos", 3);
    data_manager->RegisterData(&com_vel, VECT, "com_vel", 3);
    data_manager->RegisterData(&com_pos_des, VECT, "com_pos_des", 3);
    data_manager->RegisterData(&com_vel_des, VECT, "com_vel_des", 3);
}

void A1StateProvider::saveCurrentData(){
    for(int i=0; i<3; ++i){
        com_pos[i] = robot_->getCoMPosition()[i];
        com_vel[i] = robot_->getCoMVelocity()[i];
    }
    // for(int i=0; i<(A1::n_adof); ++i){
        // q_[i] << (robot_->getQ())[
}

