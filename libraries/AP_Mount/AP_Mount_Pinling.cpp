#include "AP_Mount_Pinling.h"
#include <GCS_MAVLink/GCS.h>

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

        //initial state'deki yaw eksenindeki sapmayı öğrenebilmek için gimbal'in açılarını istiyoruz.
        get_angles();
    }
}

void AP_Mount_Pinling::update()
{   
    // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "update__________\n\n");}
    
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // flag to trigger sending target angles to gimbal
    resend_now = false;

    calculate_angle_ef_target();

    Vector3f target_angle = _angle_ef_target_rad * RAD_TO_DEG;

    // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "_angle_ef_target_deg | x:%.3f, y:%.3f, z:%.3f\n", target_angle.x, target_angle.y, target_angle.z);}

    // Eğer derece cinsinden hedef açısı ile şu anki gimbal açısı arasında herhangi bir yönde degree_threshold dereceden fazla fark varsa gimbale hedef açıyı gönderiyoruz
    if(!(   abs(_current_imu_angle_deg.x - target_angle.x) < AP_MOUNT_PINLING_DEGREE_THRESHOLD && 
            abs(_current_imu_angle_deg.y - target_angle.y) < AP_MOUNT_PINLING_DEGREE_THRESHOLD && 
            abs(_current_stator_rel_angle_deg.z - target_angle.z) < AP_MOUNT_PINLING_DEGREE_THRESHOLD   ))
    {
        // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "degree_threshold'dan fazla fark oldu\n");}
        // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "_current_stator_rel_angle_deg | r:%.3f, p:%.3f, y:%.3f\n", _current_stator_rel_angle_deg.x, _current_stator_rel_angle_deg.y, _current_stator_rel_angle_deg.z);}
        // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "target_angle | r:%.3f, p:%.3f, y:%.3f\n", target_angle.x, target_angle.y, target_angle.z);}
        
        resend_now = true;
    }  

    // resend target angles at least once per second
    if((AP_HAL::millis() - _last_send) > AP_MOUNT_PINLING_RESEND_MS)
    {
        // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "RESEND süresi geçildi: %d ms\n", (AP_HAL::millis() - _last_send));}
        resend_now = true;
        _reply_type = ReplyType_NOREPLY;
    }

    if (resend_now && can_send()) {
        // correcting angle deviation due to wrong initial position in yaw axis
        Vector3f corrected_target_yaw_angle_deg = _angle_ef_target_rad * RAD_TO_DEG;
        corrected_target_yaw_angle_deg.z = (_angle_ef_target_rad * RAD_TO_DEG).z + (_current_imu_angle_deg.z - _current_stator_rel_angle_deg.z);

        // Vector3f corrected_target_angle_deg = corrected_target_angle_rad * RAD_TO_DEG;
        // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "corrected_target_angle_rad | r:%.3f, p:%.3f, y:%.3f\n", corrected_target_angle_deg.x, corrected_target_angle_deg.y, corrected_target_angle_deg.z);}
        control_axis(corrected_target_yaw_angle_deg * DEG_TO_RAD, false);
    }
}

void AP_Mount_Pinling::calculate_angle_ef_target(){
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
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true, true);
                // resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
}

bool AP_Mount_Pinling::can_send() {
    return _reply_type == ReplyType_NOREPLY && _port->txspace() >= sizeof(AP_Mount_Pinling::cmd_set_attitude);
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
    // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "status_msg__________\n\n");}

    get_angles();

    // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "Sending current angle (_current_stator_rel_angle_deg) to GCS: x:%.3f, y:%.3f, z:%.3f\n", _current_stator_rel_angle_deg.x*100, _current_stator_rel_angle_deg.y*100, _current_stator_rel_angle_deg.z*100);}
    mavlink_msg_mount_status_send(chan, 0, 0, _current_stator_rel_angle_deg.y*100, _current_stator_rel_angle_deg.x*100, _current_stator_rel_angle_deg.z*100);
}

// get_angles
void AP_Mount_Pinling::get_angles()
{
    // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "get_angles__________\n\n");}

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    _reply_type = ReplyType_AngleDATA;
    _reply_counter = 0;
    _reply_length = get_reply_size(_reply_type);

    cmd_query_attitude cmd;
    write_command_to_gimbal((uint8_t *)&cmd, sizeof(cmd_query_attitude), false); 

    read_incoming(true); // read the incoming messages from the gimbal
}

/*
  control_axis : send new angles to the gimbal at a fixed speed of 30 deg/s2
*/
void AP_Mount_Pinling::control_axis(const Vector3f& angle, bool target_in_degrees)
{
    // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "control_axis__________\n\n");}

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

    cmd.body.mode_roll = AP_MOUNT_PINLING_MODE_DEFAULT;
    cmd.body.mode_pitch = AP_MOUNT_PINLING_MODE_DEFAULT;
    cmd.body.mode_yaw = AP_MOUNT_PINLING_MODE_DEFAULT;

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

    // // if(isSendDebug) {gcs().send_text(MAV_SEVERITY_INFO, "target_deg | x:%.3f, y:%.3f, z:%.3f\n", target_deg.x, target_deg.y, target_deg.z);}

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

void AP_Mount_Pinling::read_incoming(bool resetCounter) {
    // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "read_incoming__________\n\n");}

    uint8_t data;
    int16_t numc;

    if (_reply_type == ReplyType_NOREPLY) {
        // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "ReplyType_NOREPLY olduğu için read_incoming'den çıkılıyor.\n");}
        return;
    }

    numc = _port->available();
    // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "Okunan byte adedi (numc): %d\n", numc);}

    if (numc <= 0 ){
        // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "Okunan veri yok, read_incoming'den çıkılıyor.\n", numc);}
        return;
    }

    if(resetCounter)
    {
        // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "Resetting counter to 0\n", numc);}
        _reply_counter = 0;
    }

    // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "Beklenen data: %d\n", _reply_length);}
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();

        // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "reply counter: %d\n", _reply_counter);}
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
    // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "parse_reply__________\n\n");}

    bool crc_ok;

    uint8_t checksum = 0; 

    // switch (_reply_type) {
    //     case ReplyType_AngleDATA:

            // check if header matches
            // uint8_t expected_header[] = {0x3E,0x3D,0x36,0x73};
            // for (uint8_t i = 0;  i != sizeof(_buffer.data.header) ; i++) {
            //     if(expected_header[i] != ((uint8_t *)&_buffer.data.body)[i])
            //     {
            //         return;
            //     }
            // }

            // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "data size: %d", sizeof(_buffer.data.body));}

            // check if checksum is correct
            for (uint8_t i = 0;  i != sizeof(_buffer.data.body) ; i++) {
                checksum += ((uint8_t *)&_buffer.data.body)[i];
            }            
            crc_ok = checksum == _buffer.data.checksum;

            // // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "crc_ok: %d | checksum: %d | data.chksm: %d\n", crc_ok, checksum, _buffer.data.checksum);}

            if (!crc_ok) {
                return;
            }

            _current_imu_angle_deg.x = VALUE_TO_DEGREE(_buffer.data.body.imu_angle_roll);
            _current_imu_angle_deg.y = VALUE_TO_DEGREE(_buffer.data.body.imu_angle_pitch);
            _current_imu_angle_deg.z = VALUE_TO_DEGREE(_buffer.data.body.imu_angle_yaw);
            // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "imu_angle_rpy | r:%.3f, p:%.3f, y:%.3f\n", _current_imu_angle_deg.x, _current_imu_angle_deg.y, _current_imu_angle_deg.z);}

            _current_rc_target_angle_deg.x = VALUE_TO_DEGREE(_buffer.data.body.rc_target_angle_roll);
            _current_rc_target_angle_deg.y = VALUE_TO_DEGREE(_buffer.data.body.rc_target_angle_pitch);
            _current_rc_target_angle_deg.z = VALUE_TO_DEGREE(_buffer.data.body.rc_target_angle_yaw);
            // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "rc_target_angle_rpy | r:%.3f, p:%.3f, y:%.3f\n", _current_rc_target_angle_deg.x, _current_rc_target_angle_deg.y, _current_rc_target_angle_deg.z);}

            _current_stator_rel_angle_deg.x = VALUE_TO_DEGREE(_buffer.data.body.stator_rel_angle_roll);
            _current_stator_rel_angle_deg.y = VALUE_TO_DEGREE(_buffer.data.body.stator_rel_angle_pitch);
            _current_stator_rel_angle_deg.z = VALUE_TO_DEGREE(_buffer.data.body.stator_rel_angle_yaw);
            // if(isGetDebug) {gcs().send_text(MAV_SEVERITY_INFO, "stator_rel_angle_rpy | r:%.3f, p:%.3f, y:%.3f\n", _current_stator_rel_angle_deg.x, _current_stator_rel_angle_deg.y, _current_stator_rel_angle_deg.z);}
                        
        //     break;
        // default:
        //     break;
    // }
}
