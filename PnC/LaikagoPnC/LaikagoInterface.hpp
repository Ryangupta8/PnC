#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"
#include "PnC/LaikagoPnC/LaikagoDefinition.hpp"

class LaikagoStateProvider;
class LaikagoStateEstimator;

class LaikagoSensorData {
   public:
    LaikagoSensorData() {
        q = Eigen::VectorXd::Zero(Laikago::n_adof);
        qdot = Eigen::VectorXd::Zero(Laikago::n_adof);
        virtual_q = Eigen::VectorXd::Zero(Laikago::n_vdof);
        virtual_qdot = Eigen::VectorXd::Zero(Laikago::n_vdof);
        front_lf_wrench = Eigen::VectorXd::Zero(6);
        front_rf_wrench = Eigen::VectorXd::Zero(6);
        rear_lf_wrench = Eigen::VectorXd::Zero(6);
        rear_rf_wrench = Eigen::VectorXd::Zero(6);
        front_rfoot_contact = false;
        front_lfoot_contact = false;
        rear_rfoot_contact = false;
        rear_lfoot_contact = false;
    }
    virtual ~LaikagoSensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd virtual_q;
    Eigen::VectorXd virtual_qdot;
    Eigen::VectorXd front_lf_wrench;
    Eigen::VectorXd front_rf_wrench;
    Eigen::VectorXd rear_lf_wrench;
    Eigen::VectorXd rear_rf_wrench;
    bool front_rfoot_contact;
    bool front_lfoot_contact;
    bool rear_rfoot_contact;
    bool rear_lfoot_contact;
};

class LaikagoCommand {
   public:
    LaikagoCommand() {
        q = Eigen::VectorXd::Zero(Laikago::n_adof);
        qdot = Eigen::VectorXd::Zero(Laikago::n_adof);
        jtrq = Eigen::VectorXd::Zero(Laikago::n_adof);
    }
    virtual ~LaikagoCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class LaikagoInterface : public EnvInterface {
   protected:
    void _ParameterSetting();

    LaikagoStateEstimator* state_estimator_;
    LaikagoStateProvider* sp_;

    void CropTorque_(LaikagoCommand*);
    bool Initialization_(LaikagoSensorData*, LaikagoCommand*);

    int count_;
    int waiting_count_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;

    double prev_planning_moment_;

   public:
    LaikagoInterface();
    virtual ~LaikagoInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);
    void Walk(double ft_length, double r_ft_width, double l_ft_width,
              double ori_inc, int num_step);

    void GetCoMTrajectory(std::vector<Eigen::VectorXd>& com_des_list);
    void GetContactSequence(std::vector<Eigen::Isometry3d>& foot_target_list);
    bool IsTrajectoryUpdated();
};
