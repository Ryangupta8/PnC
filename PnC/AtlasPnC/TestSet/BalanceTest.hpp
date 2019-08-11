#pragma once

#include <PnC/Test.hpp>

class AtlasStateProvider;

namespace BLPhase {
constexpr int BALANCE = 0;
constexpr int SWING = 1;
};  // namespace BLPhase

class BalanceTest : public Test {
   public:
    BalanceTest(RobotSystem*);
    virtual ~BalanceTest();
    virtual void TestInitialization();

   protected:
    int num_step_;
    AtlasStateProvider* sp_;
    virtual int _NextPhase(const int& phase);
    virtual void AdditionalUpdate_();
    void _SettingParameter();

    Controller* balance_ctrl_;

    YAML::Node cfg_;
};
