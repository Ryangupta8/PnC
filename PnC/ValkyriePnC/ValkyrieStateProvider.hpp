#pragma once
#include <utility>

#include <Configuration.h>
#include <Utils/IO/IOUtilities.hpp>

class RobotSystem;

class ValkyrieStateProvider {
   public:
    static ValkyrieStateProvider* getStateProvider(RobotSystem* _robot);
    ~ValkyrieStateProvider() {}

    int stance_foot;
    double curr_time;

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    Eigen::VectorXd jpos_ini;

    int b_rfoot_contact;
    int b_lfoot_contact;

    int num_step_copy;
    int phase_copy;

    // save planned result for the plot
    std::vector<Eigen::Isometry3d> foot_target_list;
    std::vector<Eigen::VectorXd> com_des_list;

   private:
    ValkyrieStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
