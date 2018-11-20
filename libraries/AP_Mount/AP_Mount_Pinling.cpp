#include "AP_Mount_Pinling.h"
// #include <AP_HAL/AP_HAL.h>
// #include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

extern const AP_HAL::HAL& hal;

AP_Mount_Pinling::AP_Mount_Pinling(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _reply_type(ReplyType_NOREPLY)
{}

void AP_Mount_Pinling::init(const AP_SerialManager& serial_manager)
{
    // check for Pinling Serial protocol 
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Pinling, 0);
    if (_port) {
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
    }
}

void AP_Mount_Pinling::update()
{   
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position. To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            // Storm32_serial.cpp benzeri kodda bu işlem en altta yapılıyor
            // control_axis(_state._retract_angles.get(), true);
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            // Storm32_serial.cpp benzeri kodda bu işlem en altta yapılıyor
            // control_axis(_state._neutral_angles.get(), true);
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            // Storm32_serial.cpp benzeri kodda bu işlem en altta yapılıyor
            // control_axis(_angle_ef_target_rad, false);
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            // Storm32_serial.cpp benzeri kodda bu işlem en altta yapılıyor
            // control_axis(_angle_ef_target_rad, false);
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true, true);
                // Storm32_serial.cpp benzeri kodda bu işlem en altta yapılıyor
                // control_axis(_angle_ef_target_rad, false);

                Vector3f target_angle = _angle_ef_target_rad * RAD_TO_DEG;
                // Eğer derece cinsinden hedef açısı ile şu anki gimbal açısı arasında herhangi bir yönde 0.1 dereceden fazla fark varsa gimbale hedef açıyı gönderiyoruz
                if(!(abs(_current_angle.x - target_angle.x) < 0.1 && abs(_current_angle.y - target_angle.y) < 0.1 && abs(_current_angle.z - target_angle.z) < 0.1))
                {
                    resend_now = true;
                }                                
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    if((AP_HAL::millis() - _last_send) > AP_MOUNT_PINLING_RESEND_MS)
    {
        resend_now = true;
        _reply_type = ReplyType_NOREPLY;
    }

    if (can_send(resend_now)) {
        if (resend_now) {
            control_axis(_angle_ef_target_rad, false);
        } else {
            get_angles();
        }
    }
}

bool AP_Mount_Pinling::can_send(bool with_control) {
    uint16_t required_tx = 1;
    if (with_control) {
        required_tx += sizeof(AP_Mount_Pinling::set_attitude_body);
    }
    return (_reply_type == ReplyType_NOREPLY) && (_port->txspace() >= required_tx);
}

uint8_t AP_Mount_Pinling::get_reply_size(ReplyType reply_type) {
    switch (reply_type) {
        case ReplyType_AngleDATA:
            return sizeof(Pinling_reply_data_struct);
            break;
        case ReplyType_camControlACK:
            return sizeof(Pinling_camera_reply_ack_struct);
            break;
        default:
            return 0;
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Pinling::has_pan_control() const
{
    // we have yaw control
    return true;
}

// set_mode - sets mount's mode
void AP_Mount_Pinling::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Pinling::status_msg(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.
    // get_angles();
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle.y*100, _current_angle.x*100, _current_angle.z*100);
}

// get_angles
void AP_Mount_Pinling::get_angles()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    _reply_type = ReplyType_AngleDATA;
    _reply_counter = 0;
    _reply_length = get_reply_size(_reply_type);

    cmd_query_attitude cmd;
    write_command_to_gimbal((uint8_t *)&cmd, sizeof(cmd_query_attitude)); 
}

/*
  control_axis : send new angles to the gimbal at a fixed speed of 30 deg/s2
*/
void AP_Mount_Pinling::control_axis(const Vector3f& angle, bool target_in_degrees)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    _reply_type = ReplyType_NOREPLY;
    _reply_counter = 0;
    _reply_length = get_reply_size(_reply_type);

    // convert to degrees if necessary
    Vector3f target_deg = angle;
    if (!target_in_degrees) {
        target_deg *= RAD_TO_DEG;
    }

    cmd_set_attitude cmd;

    cmd.header[0] = 0x3E;
    cmd.header[1] = 0x3D;
    cmd.header[2] = 0x36;
    cmd.header[3] = 0x73;

    cmd.body.mode_roll = AP_MOUNT_PINLING_MODE_ANGLE;
    cmd.body.mode_pitch = AP_MOUNT_PINLING_MODE_ANGLE;
    cmd.body.mode_yaw = AP_MOUNT_PINLING_MODE_ANGLE;

    cmd.body.speed_roll = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_PINLING_SPEED);
    cmd.body.angle_roll = DEGREE_TO_VALUE(target_deg.x);
    cmd.body.speed_pitch = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_PINLING_SPEED);
    cmd.body.angle_pitch = DEGREE_TO_VALUE(target_deg.y);
    cmd.body.speed_yaw = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_PINLING_SPEED);
    cmd.body.angle_yaw = DEGREE_TO_VALUE(target_deg.z);

    uint8_t checksum = 0; 
    for (uint8_t i = 0;  i != sizeof(set_attitude_body) ; i++) {
        checksum += ((uint8_t *)&cmd.body)[i];
    }

    cmd.checksum = checksum;

    write_command_to_gimbal((uint8_t *)&cmd, sizeof(cmd_set_attitude), true);
}

void AP_Mount_Pinling::write_command_to_gimbal(uint8_t* data, uint8_t size, bool updateLastSend)
{
    if (_port->txspace() < (size)) {
        return;
    }

    for (uint8_t i = 0; i != size; i++) {
        _port->write(data[i]);
    }

    if(updateLastSend)
    {
        // store time of send
        _last_send = AP_HAL::millis();
    }
}

void AP_Mount_Pinling::read_incoming() {
    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0 ){
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();

        if (_reply_type == ReplyType_NOREPLY) {
            continue;
        }

        _reply_length = get_reply_size(_reply_type);

        _buffer[_reply_counter++] = data;

        if (_reply_counter == _reply_length) {
            parse_reply();

            _reply_type = ReplyType_NOREPLY;
            _reply_length = 0;
            _reply_counter = 0;
        }
    }
}

void AP_Mount_Pinling::parse_reply() {
    bool crc_ok;

    uint8_t checksum = 0; 

    switch (_reply_type) {
        case ReplyType_AngleDATA:

            for (uint8_t i = 0;  i != sizeof(_buffer.data.body) ; i++) {
                checksum += ((uint8_t *)&_buffer.data.body)[i];
            }
            
            crc_ok = checksum == _buffer.data.checksum;
            if (!crc_ok) {
                return;
            }

            _current_angle.x = VALUE_TO_DEGREE(_buffer.data.body.imu_angle_roll);
            _current_angle.y = VALUE_TO_DEGREE(_buffer.data.body.imu_angle_pitch);
            _current_angle.z = VALUE_TO_DEGREE(_buffer.data.body.imu_angle_yaw);
            
            break;
        default:
            break;
    }
}
