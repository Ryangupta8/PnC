
#pragma once
#include <PnC/Test.hpp>

class RobotSystem;
class A1StateProvider;

class MoveTest : public Test {
    public: 
      MoveTest(RobotSystem* robot);
      virtual ~MoveTest();

      virtual void TestInitialization();

      void SetJointTorque(const double& value);

    protected:
      void _ParameterSetting();
      virtual int _NextPhase(const int& phase);

      Controller* joint_ctrl_;

      YAML::Node cfg_;
      A1StateProvider* sp_;

};
