#pragma once
#include <utility>

#include <Configuration.h>
#include <Utils/General/Clock.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <PnC/A1PnC/A1Definition.hpp>

class RobotSystem;

class A1StateProvider{
    public:
      static A1StateProvider* getStateProvider(RobotSystem* _robot);
      ~A1StateProvider();


      void saveCurrentData();

      Clock clock;

      double curr_time;

      Eigen::VectorXd q_;
      Eigen::VectorXd qdot_;
      Eigen::VectorXd jpos_ini_;

      int b_flfoot_contact;
      int b_frfoot_contact;
      int b_rlfoot_contact;
      int b_rrfoot_contact;

      int num_step_copy;
      int phase_copy;

      // save planned result for the plot
      std::vector<Eigen::Isometry3d> foot_target_list;
      std::vector<Eigen::VectorXd> com_des_list;

      // API related variable
      bool b_walking;
      bool b_ready_to_walk;
      double ft_length;
      double fr_ft_width;
      double fl_ft_width;
      double rr_ft_width;
      double rl_ft_width;
      double ft_ori_inc;
      int num_total_step;
      int num_residual_step;

      Eigen::VectorXd com_pos;
      Eigen::VectorXd com_vel;
      Eigen::VectorXd mom;

      Eigen::VectorXd com_pos_des;
      Eigen::VectorXd com_vel_des;
      Eigen::VectorXd mom_des;

      Eigen::VectorXd flfoot_pos;
      Eigen::VectorXd frfoot_pos;
      Eigen::VectorXd rlfoot_pos;
      Eigen::VectorXd rrfoot_pos;
      Eigen::VectorXd flfoot_vel;
      Eigen::VectorXd frfoot_vel;
      Eigen::VectorXd rlfoot_vel;
      Eigen::VectorXd rrfoot_vel;

      Eigen::VectorXd flfoot_pos_des;
      Eigen::VectorXd frfoot_pos_des;
      Eigen::VectorXd rlfoot_pos_des;
      Eigen::VectorXd rrfoot_pos_des;
      Eigen::VectorXd flfoot_vel_des;
      Eigen::VectorXd frfoot_vel_des;
      Eigen::VectorXd rlfoot_vel_des;
      Eigen::VectorXd rrfoot_vel_des;

      Eigen::VectorXd des_jacc_cmd;

    private:
      A1StateProvider(RobotSystem* _robot);
      RobotSystem* robot_;
};