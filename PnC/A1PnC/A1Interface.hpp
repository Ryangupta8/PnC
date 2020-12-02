#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"
#include "PnC/A1PnC/A1Definition.hpp"




class A1StateProvider;

class A1SensorData{
    public:
      A1SensorData(){

        q = Eigen::VectorXd::Zero(A1::n_dof);
        qdot = Eigen::VectorXd::Zero(A1::n_dof);
        q_act = Eigen::VectorXd::Zero(A1::n_adof);
        qdot_act = Eigen::VectorXd::Zero(A1::n_adof);
        virtual_q = Eigen::VectorXd::Zero(A1::n_vdof);
        virtual_qdot = Eigen::VectorXd::Zero(A1::n_vdof);

        fl_wrench = Eigen::VectorXd::Zero(6);
        fr_wrench = Eigen::VectorXd::Zero(6);
        rl_wrench = Eigen::VectorXd::Zero(6);
        rr_wrench = Eigen::VectorXd::Zero(6);

        flfoot_contact = false;
        frfoot_contact = false;
        rlfoot_contact = false;
        rrfoot_contact = false;

      }
      virtual ~A1SensorData() {}

      Eigen::VectorXd q;
      Eigen::VectorXd qdot;
      Eigen::VectorXd q_act;
      Eigen::VectorXd qdot_act;
      Eigen::VectorXd virtual_q;
      Eigen::VectorXd virtual_qdot;

      Eigen::VectorXd fl_wrench;
      Eigen::VectorXd fr_wrench;
      Eigen::VectorXd rl_wrench;
      Eigen::VectorXd rr_wrench;

      bool flfoot_contact;
      bool frfoot_contact;
      bool rlfoot_contact;
      bool rrfoot_contact;
};


class A1Command{
    public:
      A1Command(){
          q = Eigen::VectorXd::Zero(A1::n_adof);
          qdot = Eigen::VectorXd::Zero(A1::n_adof);
          jtrq = Eigen::VectorXd::Zero(A1::n_adof);
      }
      virtual ~A1Command(){}

      Eigen::VectorXd q;
      Eigen::VectorXd qdot;
      Eigen::VectorXd jtrq;
};



class A1Interface : public EnvInterface{
    protected:
      void _ParameterSetting();

      A1StateProvider* sp_;
      // A1StateEstimator* state_estimator_;

      bool Initialization_(A1SensorData*, A1Command*);

      int count_;
      int waiting_count;
      bool test_initialized;
      Eigen::VectorXd cmd_jpos_;
      Eigen::VectorXd cmd_jvel_;
      Eigen::VectorXd cmd_jtrq_;


    public:
      A1Interface();
      virtual ~A1Interface();
      virtual void getCommand(void* _sensor_data, void* _command_data);

};





