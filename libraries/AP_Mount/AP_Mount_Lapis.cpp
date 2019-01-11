#include "AP_Mount_Lapis.h"
// #include "location.h"

extern const AP_HAL::HAL& hal;

void AP_Mount_Lapis::init(const AP_SerialManager& serial_manager)
{
    // check for lapis protcol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lapis, 0))) {
        _initialised = true;
        set_motor(true);
    }
}

/*
 * set_motor will activate motors if true, and disable them if false.
 */
void AP_Mount_Lapis::set_motor(bool on)
{
    if (on) {
        lapis_power_control cmd;
        cmd.power_on = 0x01;
        gimbal_command.power = cmd;
        send_command(CMD_TO_GIMBAL_POWER);

    } else {
        lapis_power_control cmd;
        cmd.power_on = 0x00;
        gimbal_command.power = cmd;
        send_command(CMD_TO_GIMBAL_POWER);
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Lapis::has_pan_control() const
{
    return true;
}

// set_mode - sets mount's mode
void AP_Mount_Lapis::set_mode(enum MAV_MOUNT_MODE mode)
{
    // record the mode change and return success
    _state._mode = mode;
}

void AP_Mount_Lapis::collect_uav_info(){
    _current_uav_info.command_type = 0;

    float_byte _home_lat; _home_lat.f = _frontend._ahrs.get_home().lat / 1E7;
    float_byte _home_lng; _home_lng.f = _frontend._ahrs.get_home().lng / 1E7;
    float_byte _home_alt; _home_alt.f = _frontend._ahrs.get_home().alt / 100.0f;

    float_byte _uav_lat; _uav_lat.f = _frontend._current_loc.lat / 1E7;
    float_byte _uav_lng; _uav_lng.f = _frontend._current_loc.lng / 1E7;
    float_byte _uav_alt; _uav_alt.f = _frontend._current_loc.alt / 100.0f;
    float_byte _uav_yaw; _uav_yaw.f = _frontend._ahrs.yaw;

    _current_uav_info.home_lat = _home_lat;
    _current_uav_info.home_lng = _home_lng;
    _current_uav_info.home_alt = _home_alt;
    _current_uav_info.command_type |= 1<<0; //home position set

    _current_uav_info.uav_lat = _uav_lat;
    _current_uav_info.uav_lng = _uav_lng;
    _current_uav_info.uav_alt = _uav_alt;
    _current_uav_info.uav_yaw = _uav_yaw;
    _current_uav_info.command_type |= 1<<1; //uav position set
}

uint8_t AP_Mount_Lapis::calculate_angle_ef_target(){
    uint8_t _command_id = 0x00;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position. To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
                const Vector3f &target = _state._retract_angles.get();
                _angle_ef_target_rad.x = ToRad(target.x);
                _angle_ef_target_rad.y = ToRad(target.y);
                _angle_ef_target_rad.z = ToRad(target.z);

                _command_id = CMD_TO_GIMBAL_ANGLE;
            }            
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
                const Vector3f &target = _state._neutral_angles.get();
                _angle_ef_target_rad.x = ToRad(target.x);
                _angle_ef_target_rad.y = ToRad(target.y);
                _angle_ef_target_rad.z = ToRad(target.z);

                _command_id = CMD_TO_GIMBAL_ANGLE;
            }            
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            _command_id = CMD_TO_GIMBAL_ANGLE;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            {
                // update targets using pilot's rc inputs
                update_targets_from_rc();
                send_uav_info_now = true;

                _command_id = CMD_TO_GIMBAL_ANGLE;                
            }
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D && 
                   (_state._roi_target.lat != 0 && _state._roi_target.lng != 0 )) 
            {
                // Bu satırdaki hesaplama boşuna yapılıyor şu an. Sadece debug amaçlı.
                // calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true, true);                
                // //Vector3f target_deg = _angle_ef_target_rad * RAD_TO_DEG;
                // //target_deg.z += 0;

                // Umut TODO:
                // Eğer gimbal target konumundan hedefe bakmakta zorluk yaşıyorsa,
                // _angle_ef_target_rad değeri CMD_TO_GIMBAL_ANGLE komutu olarak gönderilerek de
                // gimbal'in ilgili konuma bakması sağlanabilir.

                float_byte _target_lat; _target_lat.f = _state._roi_target.lat / 1E7;
                float_byte _target_lng; _target_lng.f = _state._roi_target.lng / 1E7;
                float_byte _target_alt; _target_alt.f = _state._roi_target.alt / 100.0f;
                float_byte _target_dist; _target_dist.f = get_distance(_frontend._current_loc, _state._roi_target);
                
                _current_uav_info.target_lat = _target_lat;
                _current_uav_info.target_lng = _target_lng;
                _current_uav_info.target_alt = _target_alt;
                _current_uav_info.target_dist = _target_dist;
                _current_uav_info.command_type |= 1<<2; //target position set                
            }
            
            // _command_id = CMD_TO_GIMBAL_ANGLE;
            _command_id = CMD_TO_GIMBAL_UAV_INFO;
            
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    return _command_id;
}

bool AP_Mount_Lapis::can_send_command() {
    return _port->txspace() >= sizeof(AP_Mount_Lapis::lapis_message_to_gimbal) + sizeof(AP_Mount_Lapis::lapis_command_params);
}

// update mount position - should be called periodically
void AP_Mount_Lapis::update()
{
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // flag to trigger sending target angles to gimbal
    send_uav_info_now = false;


    collect_uav_info();
    uint8_t _command_id = calculate_angle_ef_target();

    // send target angles at least once per second
    if((AP_HAL::millis() - _last_send) > AP_MOUNT_LAPIS_RESEND_MS)
    {
        send_uav_info_now = true;
    }

    if (send_uav_info_now && can_send_command()) {        
        switch (_command_id)
        {
            case CMD_TO_GIMBAL_ANGLE:
                {
                    lapis_angle_control cmd_ang;
                    cmd_ang.command_type = 0;

                    Vector3f target_deg = _angle_ef_target_rad * RAD_TO_DEG;
                    cmd_ang.angle_yaw.f = target_deg.z;
                    cmd_ang.angle_pitch.f = target_deg.y;
                    cmd_ang.angle_roll.f = target_deg.x;

                    cmd_ang.command_type |= 1<<0; //pozisyon modu

                    gimbal_command.angle = cmd_ang;
                }
                break;

            case CMD_TO_GIMBAL_UAV_INFO:
                {
                    gimbal_command.uav_info = _current_uav_info;
                }
                break;

            default:
                break;
        }

        send_command(_command_id);
    }
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Lapis::status_msg(mavlink_channel_t chan)
{
    if (!_initialised) {
        return;
    }

    mavlink_msg_mount_status_send(
                                    chan, 0, 0, 
                                    _current_gimbal_info.gimbal_pitch.f*100.0f, 
                                    _current_gimbal_info.gimbal_roll.f*100.0f, 
                                    _current_gimbal_info.gimbal_yaw.f*100.0f
                                 );
}

/*
 send a command to the Alemox Serial API
*/
void AP_Mount_Lapis::send_command(uint8_t cmd)
{
    if (!can_send_command()) {
        return;
    }

    lapis_command_params cmd_prm;
    cmd_prm._starting_byte = CMD_STARTING_BYTE;

    uint8_t* payload;
    switch (cmd) {
        case CMD_TO_GIMBAL_POWER:
            {
                cmd_prm._command_id = CMD_TO_GIMBAL_POWER;
                cmd_prm._payload_length = sizeof(lapis_power_control);
                payload = (uint8_t *)&gimbal_command.power;
            }
            break;

        case CMD_TO_GIMBAL_ANGLE:
            {
                cmd_prm._command_id = CMD_TO_GIMBAL_ANGLE;
                cmd_prm._payload_length = sizeof(lapis_angle_control);
                payload = (uint8_t *)&gimbal_command.angle;
            }
            break;

        case CMD_TO_GIMBAL_GEOLOCK_AND_TRIM:
            {
                cmd_prm._command_id = CMD_TO_GIMBAL_GEOLOCK_AND_TRIM;
                cmd_prm._payload_length = sizeof(lapis_geolock_trim_control);
                payload = (uint8_t *)&gimbal_command.geotrim;
            }
            break;
        
        case CMD_TO_GIMBAL_UAV_INFO:
            {
                cmd_prm._command_id = CMD_TO_GIMBAL_UAV_INFO;
                cmd_prm._payload_length = sizeof(lapis_uav_info);
                payload = (uint8_t *)&gimbal_command.uav_info;
            }
            break;

        default :
            return;
            break;
    }

    cmd_prm._header_checksum = get_2s_compliment(cmd_prm._starting_byte + cmd_prm._command_id + cmd_prm._payload_length);

    _port->write( cmd_prm._starting_byte );
    _port->write( cmd_prm._command_id );
    _port->write( cmd_prm._payload_length );
    _port->write( cmd_prm._header_checksum );

    cmd_prm._body_checksum = 0;
    for (uint8_t i = 0;  i < cmd_prm._payload_length ; i++) {
        cmd_prm._body_checksum += payload[i];
        _port->write( payload[i] );
    }
    cmd_prm._body_checksum = get_2s_compliment(cmd_prm._body_checksum);
    _port->write(cmd_prm._body_checksum);

    // store time of send
    _last_send = AP_HAL::millis();
}

/*
 * detect and read the header of the incoming message from the gimbal
 */
void AP_Mount_Lapis::read_incoming()
{
    uint8_t incoming_byte;
    int16_t numc;

    numc = _port->available();

    if (numc < 0 ){
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        incoming_byte = _port->read();
        switch (_step) {
            case 0:
                if (CMD_STARTING_BYTE == incoming_byte) {
                    _step = 1;
                    _incoming_cmd = {};
                    _incoming_cmd._header_checksum = CMD_STARTING_BYTE; //reset checksum accumulator
                    _incoming_cmd._body_checksum = 0; //reset checksum accumulator
                }
                break;

            case 1: // command ID
                if(CMD_FROM_GIMBAL_INFO == incoming_byte || CMD_FROM_GIMBAL_VERSION == incoming_byte)
                {
                    _incoming_cmd._header_checksum = incoming_byte;
                    _incoming_cmd._command_id = incoming_byte;
                    _step++;
                }
                else //unknown command id
                {                    
                    _step = 0;
                    _incoming_cmd._header_checksum = 0;
                }
                break;

            case 2: // Size of the body of the message
                {
                    _incoming_cmd._header_checksum += incoming_byte;
                    _incoming_cmd._payload_length = incoming_byte;
                    _step++;
                }
                break;

            case 3:	// checksum of the header
                {
                    if (get_2s_compliment(_incoming_cmd._header_checksum) != incoming_byte) {
                        _step = 0;
                        _incoming_cmd._header_checksum = 0;
                        // checksum error
                        break;
                    }
                    _step++;
                    _payload_counter = 0;   // prepare to receive payload
                }
                break;

            case 4: // parsing body
                {
                    _incoming_cmd._body_checksum += incoming_byte;

                    uint32_t _expected_payload_size = 0;
                    if(_incoming_cmd._command_id == CMD_FROM_GIMBAL_INFO) _expected_payload_size = sizeof(lapis_gimbal_info);
                    if(_incoming_cmd._command_id == CMD_FROM_GIMBAL_VERSION) _expected_payload_size = sizeof(lapis_gimbal_version);

                    if (_payload_counter < _expected_payload_size) {
                        gimbal_response[_payload_counter] = incoming_byte;
                    }

                    if (++_payload_counter == _incoming_cmd._payload_length) {
                        _step++;
                    }
                }
                break;

            case 5: // body checksum
                {
                    _step = 0;
                    if (get_2s_compliment(_incoming_cmd._body_checksum) != incoming_byte) {
                        // checksum error
                        break;
                    }
                    parse_body(_incoming_cmd._command_id);
                }
                break;
        }
    }
}

/*
 * Parse the body of the message received from the Lapis gimbal
 */
void AP_Mount_Lapis::parse_body(uint8_t cmd_id)
{
    switch (cmd_id) {
        case CMD_FROM_GIMBAL_INFO:
            _current_gimbal_info = gimbal_response.info;
            break;

        case CMD_FROM_GIMBAL_VERSION:
            _current_gimbal_version = gimbal_response.version;
            break;

        default :
            break;
    }
}

uint8_t AP_Mount_Lapis::get_2s_compliment(uint8_t byt)
{
    return -(unsigned int) byt;
}

