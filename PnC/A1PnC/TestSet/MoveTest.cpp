#include <PnC/A1PnC/CtrlSet/JointCtrl.hpp>
#include <PnC/A1PnC/TestSet/MoveTest.hpp>
#include <PnC/A1Pnc/A1StateProvider.hpp>

MoveTest::MoveTest((RobotSystem* robot) : Test(robot) { 
    myUtils::pretty_constructor(1, "Move Test");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Scorpio/TEST/GRASPING_TEST.yaml");

    state_list_.clear();

    joint_ctrl_ = new JointCtrl(robot);

    state_list_.push_back(joint_ctrl_);

    _ParameterSetting();
    sp_ = A1StateProvider::getStateProvider(robot_);
}

MoveTest::~MoveTest(){
    delete joint_ctrl_;
}

void MoveTest::_ParameterSetting() {
    try{
        YAML::Node test_cfg = cfg_["test_configuration"];
        Eigen::VectorXd tmp_vec;
        double tmp_val;
        myUtils::readParameter(test_cfg, "moving_duration", tmp_val);
        ((JointCtrl*)joint_ctrl_)->setEndTime(tmp_val);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}


void MoveTest::SetJointTorque(const double& value){
    ((JointCtrl*)joint_ctrl_)->setTorqueValue(value);
}

