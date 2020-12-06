#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Terrain/AP_Terrain.h>
#include "AC_Acro.h"

//extern const AP_HAL::HAL& hal;

//default constructor
AC_Acro::AC_Acro(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control)
    //...
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Acro::init()
{
    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();
    
    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();
    
    //initialise state of the state machine
    _state = TrickState::Start;
    
    // capture current attitude which will be used during the TrickState::Recover stage
    const float angle_max = copter.aparm.angle_max;
    orig_attitude.x = constrain_float(_ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(_ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = _ahrs.yaw_sensor;
    
    complete = false;
}

float AC_Acro::get_altitude_error(uint16_t trick)
{
    switch(SelectedFigure)
    {
        case FLIP:
            return AC_ACRO_FLIP_MIN_ALT - _inav.get_altitude(); //altezza richiesta meno attuale, se positivo significa che siamo più bassi della quota richiesta
        break;
        
        case POWERLOOP:
            return AC_ACRO_POWERLOOP_MIN_ALT - _inav.get_altitude();
        break;
        
        case SPLIT_S:
            return AC_ACRO_SPLIT_S_MIN_ALT - _inav.get_altitude();
        break;
        
        default;
            return 20000.0f - _inav.get_altitude();
        break;
    }
}

void AC_Acro::do_selected_figure(uint16_t trick)
{   
    switch(trick)
    {
        case FLIP:
            AP_logger().Write_Event(LogEvent::TRICK_START);
            do_flip();
        break;
        
        case POWERLOOP:
            AP_logger().Write_Event(LogEvent::TRICK_START);
            do_powerloop();
        break;
        
        case SPLIT_S:
            AP_logger().Write_Event(LogEvent::TRICK_START);
            do_split_s();
        break;
        
        default:
            // error message
            //gcs().send_text(MAV_SEVERITY_WARNING, "Invalid trick");
            complete = true;
    }
}

void AC_Acro::do_flip()
{
    float throttle_out = _motors->get_throttle(); //prendere il throttle attuale, in motors_class, ci sono due funzioni quale? c'è un get_throttle_in anche in attitude_control
    
    int32_t flip_angle = _ahrs.roll_sensor;
    
    switch (_state) {
        
        case TrickState::Start:
            //under 45 degrees request 400deg/sec roll
            _attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
        
            //increase throttle
            throttle_out += FLIP_THR_INC;
        
            //beyond 45deg lean angle move to next state
            if (flip_angle >= 4500) {
                _state = TrickState::Roll;
            }
        break;
        
        case TrickState::Roll:
            // between 45deg ~ -90deg request 400deg/sec roll
            _attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
            
            // decrease throttle
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

            // beyond -90deg move on to recovery
            if ((flip_angle < 4500) && (flip_angle > -9000)) {      //flip_angle dov'è dichiarato?
                _state = TrickState::Recover;
            }
        break;
        
        case TrickState::Recover:
            // use originally captured earth-frame angle targets to recover
            _attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

            // increase throttle to gain any lost altitude
            throttle_out += FLIP_THR_INC;

            float recovery_angle;
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);

            // check for successful recovery
            if (fabsf(recovery_angle) <= TRICK_RECOVERY_ANGLE) {
                // log successful completion
                AP::logger().Write_Event(LogEvent::TRICK_END);
                
                complete = true;
            }
        break;
    }
    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}

void AC_Acro::do_powerloop()
{
    float throttle_out = _motors->get_throttle();//prendere il throttle attuale
    
    int32_t APM_angle = _ahrs.pitch_sensor;
    
    switch (_state) {
        
        case TrickState::Start:
            //between 0 and 90 degrees
            //pitch increase
            
            //throttle increase by 50%
            throttle_out += PWL_INC;
            
            if (APM_angle > 9000) {
                _state = TrickState::Loop;
            }
        break;
        
        case TrickState::Loop:
            //between 90 and 350 degrees
            //il pitch deve restare lo stesso
            
            //throttle decrese right under hover level
            throttle_out -= PWL_THR_DEC;
            
            if (APM_angle < -10000) {
                _state _ TrickState::Recover;
            }
        break;
        
        case TrickState::Recover:
            // use originally captured earth-frame angle targets to recover
            _attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

            // increase throttle to gain any lost altitude
            throttle_out += FLIP_THR_INC;

            float recovery_angle;
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);

            // check for successful recovery
            if (fabsf(recovery_angle) <= TRICK_RECOVERY_ANGLE) {
                // log successful completion
                AP::logger().Write_Event(LogEvent::TRICK_END);
                
                complete = true;
            }
        break;
    }
    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}

void AC_Acro::do_split_s()
{
    float throttle_out = _motors->get_throttle();//prendere il throttle attuale
    
    int32_t APM_pitch = ahrs.pitch_sensor;
    int32_t APM_roll = _ahrs.roll_sensor;
    
    switch (_state) {
        
        case TrickState::Start:
           //under 45 degrees request 400deg/sec roll
            _attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
        
            //increase throttle
            throttle_out += FLIP_THR_INC;
        
            //beyond 45deg lean angle move to next state
            if (APM_roll >= 4500) {
                _state = TrickState::Roll;
            }
        break;
        
        case TrickState::Roll:
            //between 45 and 180 degrees
            _attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
            
            // decrease throttle
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

            // beyond -90deg move on to recovery
            if ((APM_roll < 4500) && (APM_roll > -17999)) {      //flip_angle dov'è dichiarato?
                _state = TrickState::Loop;
            }
        break;
        
        case TrickState::Loop:
            //cut throttle
            throttle_out = 0.0f;
            
            //pitch per eseguire il loop
            
             if (APM_angle < -10000) {
                _state _ TrickState::Recover;
            }
        break;
        
        case TrickState::Recover:
            // use originally captured earth-frame angle targets to recover
            _attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

            // increase throttle to gain any lost altitude
            throttle_out += SPL_S_THR_INC;

            float recovery_angle;
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);

            // check for successful recovery
            if (fabsf(recovery_angle) <= TRICK_RECOVERY_ANGLE) {
                // log successful completion
                AP::logger().Write_Event(LogEvent::TRICK_END);
                
                complete = true;
            }
        break;
    }
    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}
