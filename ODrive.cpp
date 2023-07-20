#include "ODrive.h"

using namespace mbed;

/* Basic functions */
ODrive::ODrive(): node_id(0), can(CAN_RX,CAN_TX){}

ODrive::ODrive(uint8_t id): node_id(id), can(CAN_RX,CAN_TX){}

void ODrive::init(){
    while (!can.frequency(1000000))
        Serial.println("Bps setting....");
    while (!can.mode(CAN::Normal))
        Serial.println("Mode setting...");
    

}

uint8_t ODrive::receiveCMD(uint8_t cmd, byte res[]){
    txmsg.id=(node_id<<5)|cmd;
    txmsg.type=CANRemote;
    //memcpy(txmsg.data, req, sizeof(req));
    uint8_t result = can.write(txmsg);
    if(result==0) return CAN_FAILED; 
    while(!can.read(rxmsg) or !(rxmsg.id==txmsg.id));
    memcpy(res, rxmsg.data, sizeof(rxmsg.data));
    return CAN_OK;
}

uint8_t ODrive::sendCMD(uint8_t cmd, byte req[], size_t req_size){
    txmsg.id = (node_id<<5)|cmd;
    txmsg.type = CANData;
    memcpy(txmsg.data, req, req_size);
    uint8_t result = can.write(txmsg);
    return result;
    
}

/* Update functions */
bool ODrive::update_heartbeat(){
    return CAN_OK == receiveCMD(ODRIVE_HEARTBEAT, (byte*)&frame_read_heartbeat);
}
bool ODrive::update_motor_error(){
    return CAN_OK == receiveCMD(MOTOR_ERROR, (byte*)&frame_read_motor_error);
}
bool ODrive::update_encoder_error(){
    return CAN_OK == receiveCMD(ENCODER_ERROR, (byte*)&frame_read_encoder_error);
}
bool ODrive::update_sensorless_error(){
    return CAN_OK == receiveCMD(SENSORLESS_ERROR, (byte*)&frame_read_sensorless_error);
}
bool ODrive::update_encoder_estimate(){
    return CAN_OK == receiveCMD(ENCODER_ESTIMATE, (byte*)&frame_read_encoder_estimate);
}
bool ODrive::update_encoder_count(){
    return CAN_OK == receiveCMD(ENCODER_COUNT, (byte*)&frame_read_encoder_count);
}
bool ODrive::update_voltage(){
    return CAN_OK == receiveCMD(READ_VOLTAGE, (byte*)&frame_read_vbus);
}



/* Command functions */
bool ODrive::set_axis_state(const uint32_t& state){
    *((uint32_t*)(frame_axis_request)) = state;
    return CAN_OK == sendCMD(AXIS_REQUEST, frame_axis_request,sizeof(frame_axis_request));
}

bool ODrive::set_control_mode(const int32_t& control, const int32_t& input){
    *((int32_t*)(frame_control_mode))= control;
    *((int32_t*)(frame_control_mode+4))= input;
    return CAN_OK == sendCMD(CONTROL_MODE, frame_control_mode,sizeof(frame_control_mode));
}

// vel and tor unit is 0.001
bool ODrive::posControl(const float& pos, const int16_t& vel, const int16_t& tor){
    *((float*)(frame_pos_control))=pos;
    *((int16_t*)(frame_pos_control+4))=vel;
    *((int16_t*)(frame_pos_control+6))=tor;
    return CAN_OK == sendCMD(INPUT_POS, frame_pos_control, sizeof(frame_pos_control));
}

bool ODrive::velControl(const float& vel, const float& tor){
    *((float*)(frame_vel_control))=vel;
    *((float*)(frame_vel_control+4))=tor;
    return CAN_OK == sendCMD(INPUT_VEL, frame_vel_control, sizeof(frame_vel_control));
}

bool ODrive::torqueControl(const float& torque){
    *((float*)(frame_tor_control))=torque;
    return CAN_OK == sendCMD(INPUT_TOR, frame_tor_control, sizeof(frame_tor_control));
}

bool ODrive::set_limit(const float& vel_limit, const float& cur_limit){
    *((float*)(frame_limit_control))=vel_limit;
    *((float*)(frame_limit_control+4))=cur_limit;
    return CAN_OK == sendCMD(SET_LIMIT, frame_limit_control, sizeof(frame_limit_control)); 
}

bool ODrive::reboot(){
    return CAN_OK == sendCMD(REBOOT, frame_void, sizeof(frame_void));
}

bool ODrive::clear_error(){
    return CAN_OK == sendCMD(CLEAR_ERROR, frame_void,sizeof(frame_void));
}
