#pragma once
#include <utility>

#include <Configuration.h>
#include <PnC/RobotSystem/CentroidModel.hpp>
#include <Utils/General/Clock.hpp>
#include <Utils/IO/IOUtilities.hpp>

// how can we get the state and provide to the controller -- connect my code to this one

class RobotSystem;

class LaikagoStateProvider {
   public:
    static LaikagoStateProvider* getStateProvider(RobotSystem* _robot);
    ~LaikagoStateProvider() {}

    void saveCurrentData();

    Clock clock;

    double curr_time;
    double prev_state_machine_time;
    double planning_moment;

    int stance_foot_1;
    int stance_foot_2;
    Eigen::Isometry3d stance_foot_1_iso;
    Eigen::Isometry3d stance_foot_2_iso;
    Eigen::Isometry3d moving_foot_1_target_iso;
    Eigen::Isometry3d moving_foot_2_target_iso;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    Eigen::VectorXd jpos_ini; // Todo

    int b_front_rfoot_contact; 
    int b_front_lfoot_contact; 
    int b_rear_rfoot_contact; 
    int b_rear_lfoot_contact;

    int num_step_copy; // Todo
    int phase_copy; // Todo

    // save planned result for the plot
    std::vector<Eigen::Isometry3d> foot_target_list;
    std::vector<Eigen::VectorXd> com_des_list;

    // API related variable
    bool b_walking;
    double ft_length;
    double ft_ori_inc; // Todo
    int num_total_step; 
    int num_residual_step;
    double r_ft_width;
    double l_ft_width;
    // data manager
    Eigen::VectorXd com_pos;
    Eigen::VectorXd com_vel;
    Eigen::VectorXd mom;

    Eigen::VectorXd com_pos_des;
    Eigen::VectorXd com_vel_des;
    Eigen::VectorXd mom_des;

    Eigen::VectorXd front_rf_pos;
    Eigen::VectorXd front_rf_vel;
    Eigen::VectorXd front_lf_pos;
    Eigen::VectorXd front_lf_vel;
    Eigen::VectorXd rear_rf_pos;
    Eigen::VectorXd rear_rf_vel;
    Eigen::VectorXd rear_lf_pos;
    Eigen::VectorXd rear_lf_vel;

    Eigen::VectorXd front_rf_pos_des;
    Eigen::VectorXd front_rf_vel_des;
    Eigen::VectorXd front_lf_pos_des;
    Eigen::VectorXd front_lf_vel_des;
    Eigen::VectorXd rear_rf_pos_des;
    Eigen::VectorXd rear_rf_vel_des;
    Eigen::VectorXd rear_lf_pos_des;
    Eigen::VectorXd rear_lf_vel_des;
    
    Eigen::Quaternion<double> front_rf_ori_quat;
    Eigen::VectorXd front_rf_ang_vel;
    Eigen::Quaternion<double> front_lf_ori_quat;
    Eigen::VectorXd front_lf_ang_vel;
    Eigen::Quaternion<double> rear_rf_ori_quat;
    Eigen::VectorXd rear_rf_ang_vel;
    Eigen::Quaternion<double> rear_lf_ori_quat;
    Eigen::VectorXd rear_lf_ang_vel;

    Eigen::Quaternion<double> front_rf_ori_quat_des;
    Eigen::VectorXd front_rf_ang_vel_des;
    Eigen::Quaternion<double> front_lf_ori_quat_des;
    Eigen::VectorXd front_lf_ang_vel_des;
    Eigen::Quaternion<double> rear_rf_ori_quat_des;
    Eigen::VectorXd rear_rf_ang_vel_des;
    Eigen::Quaternion<double> rear_lf_ori_quat_des;
    Eigen::VectorXd rear_lf_ang_vel_des;
    
    Eigen::Quaternion<double> trunk_ori;
    Eigen::VectorXd trunk_ang_vel;

    Eigen::Quaternion<double> trunk_ori_des;
    Eigen::VectorXd trunk_ang_vel_des;

    // ??
    Eigen::VectorXd front_r_rf_des;
    Eigen::VectorXd front_l_rf_des;
    Eigen::VectorXd front_r_rf;
    Eigen::VectorXd front_l_rf;
    Eigen::VectorXd rear_r_rf_des;
    Eigen::VectorXd rear_l_rf_des;
    Eigen::VectorXd rear_r_rf;
    Eigen::VectorXd rear_l_rf;

    Eigen::VectorXd des_jacc_cmd;

   private:
    LaikagoStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
