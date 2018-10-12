#pragma once

#include "PnC/Test.hpp"
#include "Utils/BSplineBasic.h"
#include "PnC/FixedDracoPnC/FixedDracoInterface.hpp"

class JointTest: public Test
{
public:
    JointTest (RobotSystem* robot_);
    virtual ~JointTest ();

    virtual void getTorqueInput(void* commandData_);
    virtual void initialize();

private:
    Eigen::VectorXd mTestInitQ;
    BS_Basic<10, 3, 0, 2, 2> mSpline;
    Eigen::VectorXd mMid;
    Eigen::VectorXd mAmp;
    Eigen::VectorXd mFreq;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    double mInterpolationDuration;
    double mTestInitTime;

};