#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

ValkyrieControlArchitecture::ValkyrieControlArchitecture(RobotSystem* robot) : ControlArchitecture(robot) {
    b_first_visit_ = true;

    myUtils::pretty_constructor(1, "Valkyrie Control Architecture");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Valkyrie/TEST/BALANCE_TEST.yaml");

    sp_ = ValkyrieStateProvider::getStateProvider(robot_);
    state_ = VALKYRIE_STATES::BALANCE;

    balance_ctrl_ = new DCMBalanceCtrl(robot);

    _SettingParameter();
}

ValkyrieControlArchitecture::~ValkyrieControlArchitecture() { 
    delete balance_ctrl_; 
}

void ValkyrieControlArchitecture::ControlArchitectureInitialization() {
    balance_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["balance_ctrl"]);
}


void ValkyrieControlArchitecture::getCommand(void* _command) {
    if (b_first_visit_) {
        balance_ctrl_->firstVisit();
        b_first_visit_ = false;
    }
    balance_ctrl_->oneStep(_command);
};

void ValkyrieControlArchitecture::_SettingParameter() {
    try {
        double temp;
        Eigen::VectorXd temp_vec;

        YAML::Node test_cfg = cfg_["test_configuration"];
        myUtils::readParameter(test_cfg,"target_pos_duration",temp);
       ((DCMBalanceCtrl*)balance_ctrl_)->setDuration(temp);
        myUtils::readParameter(test_cfg,"com_pos_deviation",temp_vec);
       ((DCMBalanceCtrl*)balance_ctrl_)->setComDeviation(temp_vec);
        myUtils::readParameter(test_cfg,"amplitude",temp_vec);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}