#include <PnC/DracoPnC/PredictionModule/DCMWalkingReference.hpp>

int const DCMWalkingReference::DCM_SWING_VRP_TYPE = 0;
int const DCMWalkingReference::DCM_TRANSFER_VRP_TYPE = 1;


DCMWalkingReference::DCMWalkingReference(){
    std::cout << "[DCMWalkingReference] Constructed" << std::endl;
} 

DCMWalkingReference::~DCMWalkingReference(){

}

// Sets the desired CoM Height
void DCMWalkingReference::setCoMHeight(double z_vrp_in){
  z_vrp = z_vrp_in;
} 

void DCMWalkingReference::setInitialTime(double t_start_in){
    t_start = t_start_in;
}


void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, 
                                                        const DracoFootstep & initial_footstance,
                                                        bool clear_list){
  // Store the input footstep list
  footstep_list = input_footstep_list;
  if (input_footstep_list.size() == 0){ 
    return;
  }
  // clear DCM variables if true
  if (clear_list){
    rvrp_list.clear();
    dcm_ini_list.clear();
    dcm_eos_list.clear(); 
    dcm_ini_DS_list.clear();
    dcm_vel_ini_DS_list.clear();
    dcm_end_DS_list.clear();
    dcm_vel_end_DS_list.clear();       
  }

  // Create an rvrp for the stance leg
  Eigen::Vector3d current_rvrp(0, 0, z_vrp); // From foot local frame
  Eigen::Vector3d current_stance_rvrp(0, 0, z_vrp); // The stance leg from the foot local frame
  Eigen::Vector3d left_stance_rvrp(0, 0, z_vrp); // a left stance leg from the foot local frame
  Eigen::Vector3d right_stance_rvrp(0, 0, z_vrp); // a right stance leg from the foot local frame

  current_stance_rvrp = initial_footstance.R_ori * current_stance_rvrp + initial_footstance.position;
  left_stance_rvrp = current_stance_rvrp;
  right_stance_rvrp = current_stance_rvrp;

  // Specify that this is the eos for the previous rvrp
  rvrp_type_list.clear();
  rvrp_type_list.push_back(DCM_TRANSFER_VRP_TYPE);
  dcm_eos_list.push_back(current_stance_rvrp);

  // Add an rvrp to transfer to the stance leg
  rvrp_list.push_back(current_stance_rvrp);

  int previous_step = initial_footstance.robot_side;

  for(int i = 0; i < input_footstep_list.size(); i++){
    // initialize rvrp to [0, 0, z_vrp] in terms of the foot's local frame
    current_rvrp.setZero(); current_rvrp[2] = z_vrp;
    // Get the world frame representation
    current_rvrp = input_footstep_list[i].R_ori * current_rvrp + input_footstep_list[i].position;

    // Set the correct stance rvrp
    current_stance_rvrp = input_footstep_list[i].robot_side == DRACO_LEFT_FOOTSTEP ? right_stance_rvrp : left_stance_rvrp;
    // If this is the last step, use the average rvrp between the stance and current rvrp
    if (i == (input_footstep_list.size() - 1)){
      current_rvrp = 0.5*(current_rvrp + current_stance_rvrp);
    }

    // ----------- Begin handling same robot side footsteps -------------
    // If taking a footstep on the same side, first go to the stance foot
    if (input_footstep_list[i].robot_side == previous_step){
      // Add a new rvrp
      rvrp_type_list.push_back(DCM_TRANSFER_VRP_TYPE);
      rvrp_list.push_back(current_stance_rvrp);
    }
    else{
      // otherwise, update the correct stance to the latest rvrp
      if (input_footstep_list[i].robot_side == DRACO_LEFT_FOOTSTEP){
        left_stance_rvrp = current_rvrp;
      }else{
        right_stance_rvrp = current_rvrp;
      }
    }
    // -----------------------------------------------------------------
    // Specify that this is the eos for the previous rvrp
    rvrp_type_list.push_back(DCM_SWING_VRP_TYPE);

    // Add this rvrp to the list and also populate the DCM states
    rvrp_list.push_back(current_rvrp);

    // Update previous_step side 
    previous_step = input_footstep_list[i].robot_side;
  }
}

void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & initial_footstance, const Eigen::Vector3d & initial_rvrp){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  rvrp_list.clear();
  dcm_ini_list.clear();
  dcm_eos_list.clear(); 
  dcm_ini_DS_list.clear();
  dcm_vel_ini_DS_list.clear();
  dcm_end_DS_list.clear();
  dcm_vel_end_DS_list.clear();

  // Add the initial virtual repellant point. 
  rvrp_list.push_back(initial_rvrp); 

  // Add the remaining virtual repellant points   
  initialize_footsteps_rvrp(input_footstep_list, initial_footstance);
}

void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  Eigen::Vector3d initial_rvrp; 
  get_average_rvrp(left_footstance, right_footstance, initial_rvrp);
  // Set the stance leg
  if (input_footstep_list[0].robot_side == DRACO_LEFT_FOOTSTEP){
    initialize_footsteps_rvrp(input_footstep_list, right_footstance, initial_rvrp);
  }else{
    initialize_footsteps_rvrp(input_footstep_list, left_footstance, initial_rvrp);
  }

}

void DCMWalkingReference::initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance, const Eigen::Vector3d & initial_com){
  if (input_footstep_list.size() == 0){ 
    return;
  }

  // Set the stance leg
  if (input_footstep_list[0].robot_side == DRACO_LEFT_FOOTSTEP){
    initialize_footsteps_rvrp(input_footstep_list, right_footstance, initial_com);
  }else{
    initialize_footsteps_rvrp(input_footstep_list, left_footstance, initial_com);
  }

}

double DCMWalkingReference::get_t_step(const int & step_i){
  // Use transfer time for double support and overall step time for swing types
  if ((rvrp_type_list[step_i]) == DCMWalkingReference::DCM_TRANSFER_VRP_TYPE){
    return t_transfer + t_transfer_ds;
  }else if (rvrp_type_list[step_i] == DCMWalkingReference::DCM_SWING_VRP_TYPE){
    return t_ss + t_ds; // every swing has a double support transfer
  }
}

void DCMWalkingReference::get_average_rvrp(const DracoFootstep & footstance_1, const DracoFootstep & footstance_2, Eigen::Vector3d & average_rvrp){
  Eigen::Vector3d desired_rvrp(0, 0, z_vrp); // From foot local frame
  average_rvrp = 0.5*((footstance_1.R_ori*desired_rvrp + footstance_1.position) + (footstance_2.R_ori*desired_rvrp + footstance_2.position));
}

Eigen::Vector3d DCMWalkingReference::computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & dcm_eos_i){
  return r_vrp_d_i + std::exp(-t_step/b)*(dcm_eos_i - r_vrp_d_i);
}



void DCMWalkingReference::computeDCM_states(){
  // Clear the dcm lists
  dcm_ini_list.clear();
  dcm_eos_list.clear();
  dcm_ini_DS_list.clear(); dcm_vel_ini_DS_list.clear(); 
  dcm_end_DS_list.clear(); dcm_vel_end_DS_list.clear();  
  dcm_P.clear();

  // DCM ini and eos list is one size less than the RVRP list
  dcm_ini_list.reserve(rvrp_list.size() - 1);
  dcm_eos_list.reserve(rvrp_list.size() - 1);

  // DS DCM list is equal to the size of the rvrp  list
  dcm_ini_DS_list.reserve(rvrp_list.size()); dcm_vel_ini_DS_list.reserve(rvrp_list.size()); 
  dcm_end_DS_list.reserve(rvrp_list.size()); dcm_vel_end_DS_list.reserve(rvrp_list.size());  

  // Use backwards recursion to compute the initial and final dcm states
  double t_step = 0.0;
  for (int i = rvrp_list.size()-2; i >= 0; i--){
    // Get the t_step to use for backwards integration
    t_step = get_t_step(i);
    // Compute dcm_ini for step i
    dcm_ini_list[i] = computeDCM_ini_i(rvrp_list[i], t_step, dcm_eos_list[i]);

    // Set dcm_eos for step i-1
    if (i > 0){
      dcm_eos_list[i-1] = dcm_ini_list[i];
    }
  }
  // Last element of the DCM end of step list is equal to the last rvrp.
  dcm_eos_list.back() = rvrp_list.back();

  // Find boundary conditions for the Polynomial interpolator
  for (int i = 0; rvrp_list.size(); i++){
   dcm_ini_DS_list[i] = computeDCM_iniDS_i(i, alpha_ds*t_ds);
   dcm_end_DS_list[i] = computeDCM_eoDS_i(i, (1.0-alpha_ds)*t_ds);
   dcm_vel_ini_DS_list[i] = computeDCMvel_iniDS_i(i, alpha_ds*t_ds);
   dcm_vel_end_DS_list[i] = computeDCMvel_eoDS_i(i, (1.0-alpha_ds)*t_ds);
  }

  // Construct the polynomial matrices
}


Eigen::Vector3d DCMWalkingReference::get_DCM_exp(const int & step_index, const double & t){
  // Get t_step
  double t_step = get_t_step(step_index);
  double time = t;
  // Clamp time value
  if (t < 0){
    time = 0;
  }
  else if (t > t_step){
    time = t_step;
  }
  return rvrp_list[step_index] + std::exp( (time-t_step) / b)*(dcm_eos_list[step_index] - rvrp_list[step_index]);
}

Eigen::Vector3d DCMWalkingReference::get_DCM_vel_exp(const int & step_index, const double & t){
  // Get t_step
  double t_step = get_t_step(step_index);
  double time = t;
  // Clamp time value
  if (t < 0){
    time = 0;
  }
  else if (t > t_step){
    time = t_step;
  }
  return (1.0/b)*std::exp( (time-t_step) / b)*(dcm_eos_list[step_index] - rvrp_list[step_index]);
}


Eigen::Vector3d DCMWalkingReference::computeDCM_iniDS_i(const int & step_index, const double t_DS_ini){
  // Set Boundary condition. First element of eoDS is equal to the first element of the rvrp list
  if (step_index == 0){
    return rvrp_list.front();
  }
  return rvrp_list[step_index - 1] + std::exp(-t_DS_ini/b) * (dcm_ini_list[step_index] - rvrp_list[step_index - 1]);
}

Eigen::Vector3d DCMWalkingReference::computeDCM_eoDS_i(const int & step_index, const double t_DS_end){
  // Set Boundary condition. Last element of eoDS is equal to the last element of the rvrp list
  if (step_index == (rvrp_list.size() - 1)){
    return rvrp_list.back();
  }
  return rvrp_list[step_index] + std::exp(t_DS_end/b) * (dcm_ini_list[step_index] - rvrp_list[step_index]);
}

Eigen::Vector3d DCMWalkingReference::computeDCMvel_iniDS_i(const int & step_index, const double t_DS_ini){
  // Set Boundary condition. Velocities at the very beginning and end are always 0.0
  if (step_index == 0){
    return Eigen::Vector3d::Zero();
  }
  return (-1.0/b)*std::exp(-t_DS_ini/b) * (dcm_ini_list[step_index] - rvrp_list[step_index - 1]);
}

Eigen::Vector3d DCMWalkingReference::computeDCMvel_eoDS_i(const int & step_index, const double t_DS_end){
  // Set Boundary condition. Velocities at the very beginning and end are always 0.0
  if (step_index == (rvrp_list.size() - 1)){
    return Eigen::Vector3d::Zero();
  }
  return (1.0/b)*std::exp(t_DS_end/b) * (dcm_ini_list[step_index] - rvrp_list[step_index]);
}


Eigen::MatrixXd DCMWalkingReference::polynomialMatrix(const double Ts,
                                                      const Eigen::Vector3d & dcm_ini, const Eigen::Vector3d & dcm_vel_ini,
                                                      const Eigen::Vector3d & dcm_end, const Eigen::Vector3d & dcm_vel_end){

  Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(4,4);

  // Construct matrix mat
  mat(0,0) = 2.0/std::pow(Ts, 3);  mat(0,1) = 1.0/std::pow(Ts, 2); mat(0,2) = -2.0/std::pow(Ts, 3); mat(0,3) = 1.0/std::pow(Ts, 2);
  mat(1,0) = -3.0/std::pow(Ts, 2); mat(1,1) = -2.0/Ts;             mat(1,2) = 3.0/std::pow(Ts, 2);  mat(1,3) = -1.0/Ts;
                                   mat(2,1) = 1.0;

  Eigen::MatrixXd bound = Eigen::MatrixXd::Zero(4, 3);
  bound.row(0) = dcm_ini;
  bound.row(1) = dcm_vel_ini;
  bound.row(2) = dcm_end;
  bound.row(3) = dcm_vel_end;

  return mat*bound; 
}
