#include <Configuration.h>
#include <Utils/comm_udp.hpp>
#include <Utils/Utilities.hpp>
#include <Utils/DataManager.hpp>
#include <PnC/DracoPnC/DracoMoCapManager.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>

DracoMoCapManager::DracoMoCapManager(RobotSystem* robot):DartpThread(),
    socket_(0),
    led_pos_data_(3*NUM_MARKERS),
    led_kin_data_(3*NUM_MARKERS),
    led_pos_raw_data_(3*NUM_MARKERS),
    body_quat_(1.0, 0., 0., 0. ),
    initialization_duration_(0.5),
    b_update_call_(false)
{
    healthy_led_list_.resize(NUM_MARKERS);

    imu_body_ori_.w() = 1.0;
    imu_body_ori_.x() = 0.0;
    imu_body_ori_.y() = 0.0;
    imu_body_ori_.z() = 0.0;

    robot_ = robot;
    marker_cond_.resize(NUM_MARKERS, 0.);

    led_pos_data_.setZero();
    led_kin_data_.setZero();
    DataManager::GetDataManager()->RegisterData(&led_pos_data_, VECT, "LED_Pos", 3*NUM_MARKERS);
    // DataManager::GetDataManager()->RegisterData(&led_kin_data_, VECT, "LED_Kin_Pos", 3*NUM_MARKERS);
    // DataManager::GetDataManager()->RegisterData(&led_pos_raw_data_, VECT, "LED_Pos_Raw", 3*NUM_MARKERS);

    sp_ = DracoStateProvider::getStateProvider(robot);

    for(int i(0); i<3; ++i){
        body_led0_filter_.push_back(new digital_lp_filter(2.*M_PI*50, SERVO_RATE));
        body_led1_filter_.push_back(new digital_lp_filter(2.*M_PI*50, SERVO_RATE));
        body_led2_filter_.push_back(new digital_lp_filter(2.*M_PI*50, SERVO_RATE));
    }

    printf("[Mo Cap Manager] Constructed\n");
}

void DracoMoCapManager::run(){
    draco_message dracobip_msg;
    int count(0);


    while(true){
        ++count;
        COMM::receive_data(socket_, MOCAP_DATA_PORT,
                &dracobip_msg, sizeof(draco_message), IP_ADDRESS);

        for(int i(0); i<NUM_MARKERS; ++i){
            for(int j(0); j<3; ++j){
                led_pos_raw_data_[3*i + j] = dracobip_msg.data[3*i+j];
            }
            marker_cond_[i] = dracobip_msg.visible[i];
        }
        if( (sp_->curr_time < initialization_duration_) || b_update_call_ ){
            _CoordinateUpdate(dracobip_msg);
            b_update_call_ = false;
        }else{
            _CoordinateChange(dracobip_msg);
        }
        _UpdateLEDPosData(dracobip_msg);

        //if(count% 500 == 0){ _print_message(dracobip_msg); }
    }
}
void DracoMoCapManager::_CoordinateUpdate(draco_message & msg) {
    //double start_idx = dracobip_link::LED_BODY_0;
    //double len_to_virtual = 77;

    //for (int i = 0; i < NUM_MARKERS; ++i) {
        //if(marker_cond_[i] > 0){
            //healthy_led_list_[i][0] = msg.data[3*i];
            //healthy_led_list_[i][1] = msg.data[3*i + 1];
            //healthy_led_list_[i][2] = msg.data[3*i + 2];
        //}
    //}

    //if(!b_update_call_){
        //R_coord_ = _GetOrientation(healthy_led_list_[dracobip_link::LED_BODY_0-start_idx],
                //healthy_led_list_[dracobip_link::LED_BODY_1-start_idx],
                //healthy_led_list_[dracobip_link::LED_BODY_2-start_idx]);
         //Eigen::Matrix3d Body_rot(imu_body_ori_);
         //R_coord_ = R_coord_ * Body_rot;
    //}
    //Eigen::Vector3d local_pos;
    //offset_ = healthy_led_list_[0];
     //dynacore::pretty_print(R_coord_, std::cout, "R coord");
    //for (int i = 0; i < NUM_MARKERS; ++i) {
        //local_pos = R_coord_*(healthy_led_list_[i] - offset_);

        //msg.data[3*i] = local_pos[0];
        //msg.data[3*i + 1] = local_pos[1];
        //msg.data[3*i + 2] = local_pos[2];
    //}
}

void DracoMoCapManager::_CoordinateChange(draco_message & msg) {
    //double start_idx = dracobip_link::LED_BODY_0;
    //std::vector< Eigen::Vector3d > led_list;
    //led_list.resize(NUM_MARKERS);
    //for (int i = 0; i < NUM_MARKERS; ++i) {
        //led_list[i][0] = msg.data[3*i];
        //led_list[i][1] = msg.data[3*i + 1];
        //led_list[i][2] = msg.data[3*i + 2];
    //}

    //Eigen::Vector3d local_pos;
    //for (int i = 0; i < NUM_MARKERS; ++i) {
        //local_pos = R_coord_*(led_list[i] - offset_);

        //msg.data[3*i] = local_pos[0];
        //msg.data[3*i + 1] = local_pos[1];
        //msg.data[3*i + 2] = local_pos[2];
    //}
}

Eigen::MatrixXd DracoMoCapManager::_GetOrientation(const Eigen::Vector3d &b0_raw,
        const Eigen::Vector3d &b1_raw,
        const Eigen::Vector3d &b2_raw) {

    Eigen::Vector3d b0, b1, b2;

    for(int i(0); i<3; ++i){
        body_led0_filter_[i]->input(b0_raw[i]);
        body_led1_filter_[i]->input(b1_raw[i]);
        body_led2_filter_[i]->input(b2_raw[i]);

        b0[i] = body_led0_filter_[i]->output();
        b1[i] = body_led1_filter_[i]->output();
        b2[i] = body_led2_filter_[i]->output();
    }


    Eigen::Vector3d normal;
    //normal = (b2 - b0).cross( (b1 - b0) );
    normal = (b1 - b0).cross( (b2 - b0) );
    normal /= sqrt(normal[0]*normal[0] + normal[1]*normal[1] +
            normal[2] * normal[2]);
    Eigen::Vector3d x_coord, y_coord, z_coord;
    x_coord = normal;
    // Y
    y_coord = b1-b2;
    y_coord /= sqrt(y_coord[0]* y_coord[0] + y_coord[1]*y_coord[1] + y_coord[2]*y_coord[2]);
    // Z
    z_coord = x_coord.cross(y_coord);
    z_coord /= sqrt(z_coord[0]*z_coord[0] + z_coord[1]*z_coord[1] +
            z_coord[2] * z_coord[2]);
    Eigen::MatrixXd R(3, 3);
    R << x_coord[0], y_coord[0], z_coord[0],
    x_coord[1], y_coord[1], z_coord[1],
    x_coord[2], y_coord[2], z_coord[2];


    Eigen::Matrix3d R_mat = R;
    Eigen::Quaternion<double> quat(R_mat);
    body_quat_ = quat;

    return R.transpose();
}

void DracoMoCapManager::_Print_message(const draco_message & msg){
    for(int i(0); i < NUM_MARKERS; ++i){
        printf("%d th LED data (cond, x, y, z): %d, (%f, %f, %f) \n", i,
                msg.visible[i],
                msg.data[3*i],
                msg.data[3*i+1],
                msg.data[3*i+2]);
        if(i == (NUM_MARKERS - 1)) {
            // printf("size: %lu\n", sizeof(draco_message));
            printf("\n");
        }
    }
}

void DracoMoCapManager::_UpdateLEDPosData(const draco_message & msg){

    led_kin_data_ = sp_->led_kin_data;

    int led_number(0);
    for(int i(0); i<3*NUM_MARKERS; ++i){
        if (msg.visible[led_number] > 0) {
            led_pos_data_[i] = msg.data[i] * 0.001;
        }

        if (i%3 == 2) {
            ++led_number;
        }
    }
}