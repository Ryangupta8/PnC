#ifndef DRACO_FOOTSTEP_OBJECT_H
#define DRACO_FOOTSTEP_OBJECT_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

#define DRACO_LEFT_FOOTSTEP 0
#define DRACO_RIGHT_FOOTSTEP 1
#define DRACO_MID_FOOTSTEP 2

// Data container for a footstep object
 

class DracoFootstep{
public:
  DracoFootstep();
  DracoFootstep(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in);

  ~DracoFootstep(); 

  // Position and orientation of the sole of the footstep
  Eigen::Vector3d position;
  Eigen::Quaternion<double> orientation;

  Eigen::Matrix3d R_ori;

  // Setters
  void setPosOriSide(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in, const int & robot_side_in);
  void setPosOri(const Eigen::Vector3d & pos_in, const Eigen::Quaterniond & quat_in);
  void setRightSide();
  void setLeftSide();
  void setMidFoot();

  // Left or right side
  int robot_side;

  void computeMidfeet(const DracoFootstep & footstep1, const DracoFootstep & footstep2, DracoFootstep & midfeet);

  int getSide();
  void printInfo();

  // length of the soles
  double sole_length = 0.2;

  // location of the front and back contacts from the center of the foot.
  double foot_front = 0.015;
  double foot_back = 0.015;

  std::vector<Eigen::Vector3d> local_contact_point_list;
  std::vector<Eigen::Vector3d> global_contact_point_list;


private:
  void common_initialization();
  void updateContactLocations();

};

#endif