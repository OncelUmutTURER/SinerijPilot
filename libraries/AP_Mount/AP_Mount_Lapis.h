/*
  Lapis Serial Protocol controlled mount backend class
*/
#pragma once

#include "AP_Mount.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Mount_Backend.h"

//definition of the commands id for the Lapis Serial Protocol
#define CMD_TO_GIMBAL_POWER             0x50
#define CMD_TO_GIMBAL_ANGLE             0x52
#define CMD_TO_GIMBAL_GEOLOCK_AND_TRIM  0x56
#define CMD_TO_GIMBAL_UAV_INFO          0x57 // Frequency: 1 Hz
#define CMD_FROM_GIMBAL_INFO            0xA2 // Frequency: 6 Hz
#define CMD_FROM_GIMBAL_VERSION         0xA3 // Frequency: 1 Hz

#define AP_MOUNT_LAPIS_RESEND_MS        1000 // resend angle targets to gimbal once per second
#define CMD_STARTING_BYTE               0xBB

class AP_Mount_Lapis : public AP_Mount_Backend
{
public:
    //constructor
    AP_Mount_Lapis(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance)
    {}

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode) ;

    // status_msg - called to allow mounts to send their status to GCS via MAVLink
    virtual void status_msg(mavlink_channel_t chan) ;

private:

    uint8_t calculate_angle_ef_target();

    // set_motor will activate motors if true, and disable them if false
    void set_motor(bool on);

    // write_params - write new parameters to the gimbal settings
    void write_params();

    // Lapis Serial Protocol reading part implementation
    // send_command - send a command to the Lapis Serial API
    void send_command(uint8_t cmd);

    // read_incoming - detect and read the header of the incoming message from the gimbal
    void read_incoming();

    // Parse the body of the message received from the Lapis gimbal
    void parse_body(uint8_t cmd_id);

    bool can_send_command();

    void collect_uav_info();

    uint8_t get_2s_compliment(uint8_t byt);


    // structure for the Serial Protocol

    union float_byte {
        float f;
        u_int8_t b[4];
    };

    // CMD_TO_GIMBAL_POWER
    struct PACKED lapis_power_control {
        uint8_t power_on;   //0x00 = Güç kapat, 0x01 = Güç aç
    };

    // CMD_TO_GIMBAL_ANGLE
    struct PACKED lapis_angle_control {
        int16_t command_type;       //16 bit    //LSB-> 0:pozisyon, 1:hız, 2:pilot mod, 3: koruma mod
        float_byte angle_yaw;       //degree
        float_byte angle_pitch;     //degree
        float_byte angle_roll;      //degree
        float_byte speed_yaw;       //degree/sec
        float_byte speed_pitch;     //degree/sec
        float_byte speed_roll;      //degree/sec
    };

    // CMD_TO_GIMBAL_GEOLOCK_AND_TRIM
    struct PACKED lapis_geolock_trim_control {
        int16_t command_type;       //16 bit    //LSB-> 0:Geo-Lock Durum, 1:Trim Durum
        uint8_t geo_lock;       //0x00 = Off 0x01 = On
        int8_t trim;            //-1 = Sola trim 1= Sağa trim 0 = Mevcut trim kaldır
    };

    // Frequency: 1 Hz
    // CMD_TO_GIMBAL_UAV_INFO 
    struct PACKED lapis_uav_info {        
        float_byte home_lat;        //degree
        float_byte home_lng;        //degree
        float_byte home_alt;        //m
        float_byte uav_lat;         //degree
        float_byte uav_lng;         //degree
        float_byte uav_alt;         //m
        float_byte uav_yaw;         //degree        
        float_byte target_lat;      //degree
        float_byte target_lng;      //degree
        float_byte target_alt;      //m
        float_byte target_dist;     //m         //umut sor: 3d dist mi 2d mi?
        int16_t command_type;       //16 bit    //LSB-> 0:home position, 1:uav position, 2:target position
    };

    // Frequency: 6 Hz
    // CMD_FROM_GIMBAL_INFO
    struct PACKED lapis_gimbal_info {
        float_byte gimbal_roll;     //degree
        float_byte gimbal_pitch;    //degree
        float_byte gimbal_yaw;      //degree
        float_byte target_lat;      //degree
        float_byte target_lng;      //degree
        float_byte target_alt;      //m
        float_byte target_dist;     //m
        float_byte gimbal_true_yaw; //degree    // ilk 2 byte: int16_t command_type;       //16 bit    //LSB-> 0:geo_lock açık, 1:sağa trim var, 2:sola trim var, 3:Hedef Nokta Kaynağı (1=YKİ, 0=Gimbal)
    };

    // Frequency: 1 Hz
    // CMD_FROM_GIMBAL_VERSION
    struct PACKED lapis_gimbal_version {
        uint8_t firmware_version;
        uint8_t hardware_version;
    };

    union PACKED lapis_message_to_gimbal {
        DEFINE_BYTE_ARRAY_METHODS
        lapis_power_control power;
        lapis_angle_control angle;
        lapis_geolock_trim_control geotrim;
        lapis_uav_info uav_info;
    } gimbal_command;

    union PACKED lapis_message_from_gimbal {
        DEFINE_BYTE_ARRAY_METHODS
        lapis_gimbal_info info;
        lapis_gimbal_version version;
    } gimbal_response;

    // Serial Protocol Variables
    struct PACKED lapis_command_params {
        uint8_t _starting_byte = CMD_STARTING_BYTE;
        uint8_t _command_id;
        uint8_t _payload_length;
        uint8_t _header_checksum;

        uint8_t _body_checksum;
    };

    uint8_t _step;              //used for parsing gimbal response
    uint8_t _payload_counter;   //used for parsing gimbal response

    // current uav and gimbal states
    lapis_uav_info _current_uav_info;
    lapis_gimbal_info _current_gimbal_info;
    lapis_gimbal_version _current_gimbal_version;

    AP_HAL::UARTDriver *_port;

    bool _initialised : 1;          // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    bool send_uav_info_now = false; // send target angles to mount

    uint8_t _reply_length;
    uint8_t _reply_counter;
};
