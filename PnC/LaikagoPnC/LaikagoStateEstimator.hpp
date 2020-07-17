#pragma once

#include <Configuration.h>
#include <Eigen/Dense>

class LaikagoStateProvider;
class RobotSystem;
class LaikagoSensorData;

class LaikagoStateEstimator {
   public:
    LaikagoStateEstimator(RobotSystem* robot);
    ~LaikagoStateEstimator();

    void Initialization(LaikagoSensorData*);
    void Update(LaikagoSensorData*);

   protected:
    LaikagoStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;

    void _JointUpdate(LaikagoSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(LaikagoSensorData* data);
};
