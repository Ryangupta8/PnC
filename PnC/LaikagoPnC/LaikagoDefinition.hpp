#pragma once
namespace Laikago {
constexpr int n_bodynode = 29;
constexpr int n_dof = 18;
constexpr int n_vdof = 6;
constexpr int n_adof = 12; 

}  // namespace Valkyrie

namespace LaikagoBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int trunk = 5;
constexpr int FR_hip = 6;
constexpr int FR_thigh_shoulder = 7;
constexpr int FR_thigh = 8;
constexpr int FR_calf = 9;
constexpr int FR_foot = 10;
constexpr int FL_hip = 11;
constexpr int FL_thigh_shoulder = 12;
constexpr int FL_thigh = 13;
constexpr int FL_calf = 14;
constexpr int FL_foot = 15;
constexpr int RR_hip = 16;
constexpr int RR_thigh_shoulder = 17;
constexpr int RR_thigh = 18;
constexpr int RR_calf = 19;
constexpr int RR_foot = 20;
constexpr int RL_hip = 21;
constexpr int RL_thigh_shoulder = 22;
constexpr int RL_thigh = 23;
constexpr int RL_calf = 24;
constexpr int RL_foot = 25;
constexpr int camera_link = 26;
constexpr int camera_rgb_frame = 27;
constexpr int camera_rgb_optical_frame = 28;
}  // namespace LaikagoBodyNode

namespace LaikagoDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int trunk = 5;
constexpr int FR_hip_joint = 6;
constexpr int FR_thigh_joint = 7;
constexpr int FR_calf_joint = 8;
constexpr int FL_hip_joint = 9;
constexpr int FL_thigh_joint = 10;
constexpr int FL_calf_joint = 11;
constexpr int RR_hip_joint = 12;
constexpr int RR_thigh_joint = 13;
constexpr int RR_calf_joint = 14;
constexpr int RL_hip_joint = 15;
constexpr int RL_thigh_joint = 16;
constexpr int RL_calf_joint = 17;
}  // namespace LaikagoDoF

namespace LaikagoAux {
constexpr double servo_rate = 0.001; // Todo
}
