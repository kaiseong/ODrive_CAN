#ifndef ODRIVE_H
#define ODRIVE_H

#include "mbed.h"
#include "interface.h"
using namespace mbed;

/* result communication */
#define CAN_FAILED                          0
#define CAN_OK                              1

/* pin set */
#define CAN_RX                              PB_5
#define CAN_TX                              PB_13

/* command list */
#define ODRIVE_HEARTBEAT                    0x01
#define MOTOR_ERROR                         0x03
#define ENCODER_ERROR                       0x04
#define SENSORLESS_ERROR                    0x05
#define AXIS_REQUEST                        0x07
#define ENCODER_ESTIMATE                    0x09
#define ENCODER_COUNT                       0x0A    
#define CONTROL_MODE                        0x0B
#define INPUT_POS                           0x0C
#define INPUT_VEL                           0x0D    
#define INPUT_TOR                           0x0E
#define SET_LIMIT                           0x0F
#define START_ANTICOGGING                   0x10
#define REBOOT                              0x16
#define READ_VOLTAGE                        0x17
#define CLEAR_ERROR                         0x18 




class ODrive{
private:
    CAN can;
    CANMessage rxmsg;
    CANMessage txmsg;
    uint8_t node_id;
    uint8_t sendCMD(uint8_t cmd, byte req[], size_t req_size);
    uint8_t receiveCMD(uint8_t cmd, byte res[]);

    byte frame_read_heartbeat[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_motor_error[8]      = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_encoder_error[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_sensorless_error[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_encoder_estimate[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_encoder_count[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_read_vbus[8]             = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_control_mode[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_axis_request[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_vel_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_pos_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_tor_control[8]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_limit_control[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte frame_void[8]                  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

public:
    /* Basic */
    ODrive();
    ODrive(uint8_t id);
    void init();

    /* Read function */
    bool update_heartbeat();
    bool update_motor_error();
    bool update_encoder_error();
    bool update_sensorless_error();
    bool update_encoder_estimate();
    bool update_encoder_count();
    bool update_voltage();

    /* Get motor states */
    inline const uint32_t& get_axis_error()         { return *((uint32_t*)(frame_read_heartbeat));  }
    inline const uint8_t& get_axis_state()          { return *((uint8_t*)(frame_read_heartbeat+4)); }
    inline const uint64_t& get_motor_error()        { return *((uint64_t*)(frame_read_motor_error)); }
    inline const uint32_t& get_encoder_error()      { return *((uint32_t*)(frame_read_encoder_count)); }
    inline const uint32_t& get_sensorless_error()   { return *((uint32_t*)(frame_read_sensorless_error)); }
    inline const float& get_position()          { return *((float*)(frame_read_encoder_estimate)); }
    inline const float& get_velocity()          { return *((float*)(frame_read_encoder_estimate+4)); }
    inline const float& get_voltage()           { return *((float*)(frame_read_vbus)); }
    
    /* Command function */
    bool set_axis_state(const uint32_t& state);
    bool set_control_mode(const int32_t& control, const int32_t& input);
    bool posControl(const float& pos, const int16_t& vel, const int16_t& tor);
    bool velControl(const float& vel, const float& tor);
    bool torqueControl(const float& torque);
    bool set_limit(const float& vel_limit, const float& cur_limit);
    bool reboot();
    bool clear_error();
};

#endif