/*
  PINLING SERIAL GIMBAL PROTOCOL controlled mount backend class
*/
#pragma once

#include "AP_Mount.h"
#include <AP_HAL/AP_HAL.h>
#include "AP_Mount_Backend.h"

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>

#define AP_MOUNT_PINLING_SPEED 30 // degree/s2

#define AP_MOUNT_PINLING_MODE_NO_CONTROL 0
#define AP_MOUNT_PINLING_MODE_SPEED 1
#define AP_MOUNT_PINLING_MODE_ANGLE 2
#define AP_MOUNT_PINLING_MODE_SPEED_ANGLE 3
#define AP_MOUNT_PINLING_MODE_RC 4
#define AP_MOUNT_PINLING_MODE_ANGLE_REL_FRAME 5

#define VALUE_TO_DEGREE(d) ((float)((d * 720) >> 15))
#define DEGREE_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.02197265625f)))
#define DEGREE_PER_SEC_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.1220740379f)))

class AP_Mount_Pinling : public AP_Mount_Backend 
{
public:
    //constructor
    AP_Mount_Pinling(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance)
    {}

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

    // get_angles
    void get_angles();

    // control_axis - send new angles to the gimbal at a fixed speed of 30 deg/s
    void control_axis(const Vector3f& angle , bool targets_in_degrees);

    // read_incoming - detect and read the header of the incoming message from the gimbal
    void read_incoming();
    void parse_reply();

    // send_command - send any command via serial
    void send_command(uint8_t* data, uint8_t size);

    // struct PACKED cmd_set_address {
    //     uint8_t byte_1=0x88;
    //     uint8_t byte_2=0x30;
    //     uint8_t byte_3=0x01;
    //     uint8_t byte_4=0xFF;
    // };

    struct PACKED cmd_set_zoom_in {       
        uint8_t byte_1=0xFF;
        uint8_t byte_2=0x01;
        uint8_t byte_3=0x20;
        uint8_t byte_4=0x00;
        uint8_t byte_5=0x00;
        uint8_t byte_6=0x21;

    };

    struct PACKED cmd_set_zoom_out {
        uint8_t byte_1=0xFF;
        uint8_t byte_2=0x01;
        uint8_t byte_3=0x40;
        uint8_t byte_4=0x00;
        uint8_t byte_5=0x00;
        uint8_t byte_6=0x41;
    };

    struct PACKED cmd_set_zoom_stop {
        uint8_t byte_1=0xFF;
        uint8_t byte_2=0x01;
        uint8_t byte_3=0x00;
        uint8_t byte_4=0x00;
        uint8_t byte_5=0x00;
        uint8_t byte_6=0x01;
    };

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

        CS      : checksum:checksum is calculated as a sum of all data bytes (from ‘RM’ to ‘YAH’) modulo 256
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
    struct PACKED query_attitude_response {
        uint8_t header[4]={0x3E,0x3D,0x36,0x73};
        query_attitude_response_body body;
        uint8_t checksum;
    } gimbal_response;

    // internal variables
    AP_HAL::UARTDriver *_port;

    bool _initialised : 1;

    uint8_t _reply_counter;

    // keep the last _current_angle values
    Vector3f _current_angle;
};

