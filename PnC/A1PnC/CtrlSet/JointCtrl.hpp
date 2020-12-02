
#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;

class A1StateProvider;
class Task;
class WBC;

class JointCtrl : public Controller {
    public:
      JointCtrl(RobotSystem* _robot);
      virtual ~JointCtrl();

      void setEndTime(double time) { end_time_ = time; }

      void setTorqueValue(const double& value) {
          torque_value = value;
      }

    protected:
      double end_time;
      double torque_value;

