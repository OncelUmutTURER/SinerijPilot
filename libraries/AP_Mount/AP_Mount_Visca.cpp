#include "AP_Mount_Visca.h"
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

extern const AP_HAL::HAL& hal;

void AP_Mount_Visca::init(const AP_SerialManager& serial_manager)
{
    // check for Visca protocol 
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Visca, 0))) {
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());

        // initialize camera address
        // cmd_set_address cmd;
        // send_command((uint8_t *)&cmd, sizeof(cmd_set_address));
    }
}

void AP_Mount_Visca::update()
{   
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            control_axis(_state._retract_angles.get(), true);
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            control_axis(_state._neutral_angles.get(), true);
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            // control_axis(_angle_ef_target_rad, false);
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            control_axis(_angle_ef_target_rad, false);
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, false);
                control_axis(_angle_ef_target_rad, false);
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Visca::has_pan_control() const
{
    // we have yaw control
    return true;
}

// set_mode - sets mount's mode
void AP_Mount_Visca::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Visca::status_msg(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.
    get_angles();
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle.y*100, _current_angle.x*100, _current_angle.z*100);
}

// get_angles
void AP_Mount_Visca::get_angles()
{
    cmd_query_attitude cmd;
    send_command((uint8_t *)&cmd, sizeof(cmd_query_attitude)); 
}

/*
  control_axis : send new angles to the gimbal at a fixed speed of 30 deg/s2
*/
void AP_Mount_Visca::control_axis(const Vector3f& angle, bool target_in_degrees)
{
    // convert to degrees if necessary
    Vector3f target_deg = angle;
    if (!target_in_degrees) {
        target_deg *= RAD_TO_DEG;
    }

    cmd_set_attitude cmd;

    cmd.body.mode_roll = AP_MOUNT_VISCA_MODE_ANGLE;
    cmd.body.mode_pitch = AP_MOUNT_VISCA_MODE_ANGLE;
    cmd.body.mode_yaw = AP_MOUNT_VISCA_MODE_ANGLE;

    cmd.body.speed_roll = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_VISCA_SPEED);
    cmd.body.angle_roll = DEGREE_TO_VALUE(target_deg.x);
    cmd.body.speed_pitch = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_VISCA_SPEED);
    cmd.body.angle_pitch = DEGREE_TO_VALUE(target_deg.y);
    cmd.body.speed_yaw = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_VISCA_SPEED);
    cmd.body.angle_yaw = DEGREE_TO_VALUE(target_deg.z);

    uint8_t checksum = 0; 

    for (uint8_t i = 0;  i != sizeof(set_attitude_body) ; i++) {
        checksum += ((uint8_t *)&cmd.body)[i];
    }

    cmd.checksum = checksum;

    send_command((uint8_t *)&cmd, sizeof(cmd_set_attitude));
}

void AP_Mount_Visca::send_command(uint8_t* data, uint8_t size)
{
    if (_port->txspace() < (size)) {
        return;
    }

    for (uint8_t i = 0; i != size; i++) {
        _port->write(data[i]);
    }
}

void AP_Mount_Visca::read_incoming() {

    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0 ){
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();

        ((uint8_t *)&gimbal_response)[_reply_counter++] = data;
        if (_reply_counter == sizeof(query_attitude_response)) {
            parse_reply();
            _reply_counter = 0;
        }
    }
}

void AP_Mount_Visca::parse_reply() {
    bool crc_ok;

    uint8_t checksum = 0; 

    for (uint8_t i = 0;  i != sizeof(gimbal_response.body) ; i++) {
        checksum += ((uint8_t *)&gimbal_response.body)[i];
    }
    
    crc_ok = checksum == gimbal_response.checksum;
    if (!crc_ok) {
        return;
    }

    // _current_angle.x = gimbal_response.body.imu1_roll;
    // _current_angle.y = gimbal_response.body.imu1_pitch;
    // _current_angle.z = gimbal_response.body.imu1_yaw;
}
