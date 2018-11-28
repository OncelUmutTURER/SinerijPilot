/*
  PINLING SERIAL GIMBAL PROTOCOL controlled mount backend class
*/
#pragma once

#include "AP_Mount_Backend.h"

#define AP_MOUNT_PINLING_RESEND_MS   1000   // resend angle targets to gimbal once per second
#define AP_MOUNT_PINLING_SPEED 0            // degree/s2 //0: default gimbal speed

#define AP_MOUNT_PINLING_MODE_NO_CONTROL 0
#define AP_MOUNT_PINLING_MODE_SPEED 1
#define AP_MOUNT_PINLING_MODE_ANGLE 2
#define AP_MOUNT_PINLING_MODE_SPEED_ANGLE 3
#define AP_MOUNT_PINLING_MODE_RC 4
#define AP_MOUNT_PINLING_MODE_ANGLE_REL_FRAME 5
#define AP_MOUNT_PINLING_MODE_DEFAULT 2

#define VALUE_TO_DEGREE(d) ((float)((d * 720) >> 15))
#define DEGREE_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.02197265625f)))
#define DEGREE_PER_SEC_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.1220740379f)))

class AP_Mount_Pinling : public AP_Mount_Backend 
{

public:
    //constructor
    AP_Mount_Pinling(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position & camera control comands - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS via MAVLink
    virtual void status_msg(mavlink_channel_t chan);

private:

    void calculate_angle_ef_target();

    // get_angles
    void get_angles();

    // control_axis - send new angles to the gimbal at a fixed speed of 30 deg/s
    void control_axis(const Vector3f& angle , bool targets_in_degrees);

    // read_incoming - detect and read the header of the incoming message from the gimbal
    void read_incoming(bool resetCounter = false);
    void parse_reply();

    enum ReplyType {
        ReplyType_NOREPLY = 0,
        ReplyType_AngleDATA,
        ReplyType_camControlACK
    };

    // send_command - send any command via serial
    void write_command_to_gimbal(uint8_t* data, uint8_t size, bool updateLastSend=false);

    //void add_next_reply(ReplyType reply_type);
    uint8_t get_reply_size(ReplyType reply_type);
    bool can_send();

    struct PACKED set_attitude_body {
        uint8_t mode_roll;
        uint8_t mode_pitch;
        uint8_t mode_yaw;
        int16_t speed_roll;
        int16_t angle_roll;
        int16_t speed_pitch;
        int16_t angle_pitch;
        int16_t speed_yaw;
        int16_t angle_yaw;
    };

    /*
        FF 01 0F 10 { ...body... } CS
        { ...body... } : RM PM YM RSL RSH RAL RAH PSL PSH PAL PAH YSL YSH YAL YAH

        R       : ROLL
        P       : PITCH
        Y       : YAW

        M       : control mode: 00=no_control , 01=speed_mode, 02 = angle_mode,03= speed_angle_mode, 04= RC_mode,05=mode_angle_rel_frame. (please use RC_mode, typically)
        SL SH   : speed (units: 0.1220740379 degree/sec)
        AL AH   : angle (units: 0.02197265625 degree)

        CS      : checksum: checksum is calculated as a sum of all data bytes (from ‘RM’ to ‘YAH’) modulo 256
    */
    struct PACKED cmd_set_attitude {
        uint8_t header[4]={0xFF,0x01,0x0F,0x10};
        set_attitude_body body;
        uint8_t checksum;
    };

    struct PACKED cmd_query_attitude {
        uint8_t byte_1=0x3E;
        uint8_t byte_2=0x3D;
        uint8_t byte_3=0x00;
        uint8_t byte_4=0x3D;
        uint8_t byte_5=0x00;
    };

    struct PACKED query_attitude_response_body {
        int16_t imu_angle_roll;
        int16_t rc_target_angle_roll;
        int32_t stator_rel_angle_roll;
        uint8_t reserved_bytes_roll[10];

        int16_t imu_angle_pitch;
        int16_t rc_target_angle_pitch;
        int32_t stator_rel_angle_pitch;
        uint8_t reserved_bytes_pitch[10];

        int16_t imu_angle_yaw;
        int16_t rc_target_angle_yaw;
        int32_t stator_rel_angle_yaw;
        uint8_t reserved_bytes_yaw[10];
    };

    /*
        3e 3d 36 73 { ...body... } cs
        { ...body... } :    ROLL_IMU_ANGLE[2] ROLL_RC_TARGET_ANGLE[2] ROLL_STATOR_REL_ANGLE[4] RES_BYTES[10]
                            PITCH_IMU_ANGLE[2] PITCH_RC_TARGET_ANGLE[2] PITCH_STATOR_REL_ANGLE[4] RES_BYTES[10]
                            YAW_IMU_ANGLE[2] YAW_RC_TARGET_ANGLE[2] YAW_STATOR_REL_ANGLE[4] RES_BYTES[10]
    */
    struct PACKED Pinling_reply_data_struct {
        uint8_t header[4];//={0x3E,0x3D,0x36,0x73};
        query_attitude_response_body body;
        uint8_t checksum;
    };

    struct PACKED Pinling_camera_reply_ack_struct {
        uint8_t feedback[8];
    };

    union PACKED Pinling_reply {
        DEFINE_BYTE_ARRAY_METHODS
        Pinling_reply_data_struct data;
        Pinling_camera_reply_ack_struct ack;
    } _buffer;
    
    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised : 1;          // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    bool resend_now = false;        // resend target angles to mount

    uint8_t _reply_length;
    uint8_t _reply_counter;
    ReplyType _reply_type;

    // const bool isSendDebug = false;
    // const bool isGetDebug = false;
    const char debug_prefix[5] = {'D','B','G',':',' '};
    const float AP_MOUNT_PINLING_DEGREE_THRESHOLD = 1; // derece cinsinden hedef nokta ile baktığı nokta arasında bu dereceden az fark varsa komut göndermez

    // keep the last _current_angle values
    Vector3f _current_imu_angle_deg;
    Vector3f _current_rc_target_angle_deg;
    Vector3f _current_stator_rel_angle_deg;
};

