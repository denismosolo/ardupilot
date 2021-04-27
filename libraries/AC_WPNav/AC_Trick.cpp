#include <AP_HAL/AP_HAL.h>
#include "AC_Trick.h"

extern const AP_HAL::HAL& hal;

#define TRICK_DEFAULT 2

const AP_Param::GroupInfo AC_Trick::var_info[] = {
    // @Param: TRICK
    // @DisplayName: Trick da eseguire
    // @Description: Trick da eseguire, default 2->flip
    // @Units: -
    // @Range: 0 2
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TRICK", 1, AC_Trick, _trick, TRICK_DEFAULT),
    
    AP_GROUPEND
};

//costruttore di default
AC_Trick::AC_Trick(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_AttitudeControl& attitude_control, AP_Motors& motors, const AP_Vehicle::MultiCopter& aparm) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control),
    _motors(motors),
    _aparm(aparm)
{
    AP_Param::setup_object_defaults(this, var_info);
    printf("\nchiamo il costruttore\n\n");
}

//init
void AC_Trick::init()
{
    printf("\ninizializzo la state machine\n");
    
    //catturo la configurazione iniziale per ripristinarla alla fine del trick
    const float angle_max = _aparm.angle_max;
    orig_attitude.x = constrain_float(_ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(_ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = _ahrs.yaw_sensor;
    
    //catturo il livello di hover del throttle
    _thr_hover = _motors.get_throttle_hover();
    
    //inizializzo lo stato della state machine
    _state = TrickState::Start;
    
    //faccio partire il timer
    _start_time_ms = millis();
    
    //complete inizializzato a falso - diventerà vero a figura eseguita
    _complete = false;
}

//mi scrivo la figura da eseguire
void AC_Trick::set_fig_from_cmd(uint16_t trick)
{
    _trick = trick;
    //printf("\nHO RICEVUTO QUESTA FIGURA: %d\n HO SCRITTO QUESTA FIGURA: %d", trick, _trick_);
}

//selettore di figura
void AC_Trick::do_selected_figure()
{
    printf("\neseguo la figura\n");
    switch(_trick)
    {
        case FLIP:
            printf("\neseguo un flip\n");
            do_flip();
            break;
        case POWERLOOP:
            printf("\neseguo un powerloop\n");
            do_powerloop();
            break;
        case SPLIT_S:
            printf("\neseguo una split_S\n");
            do_split_s();
            break;
        default:
            //segnalo che non ho riconosciuto la figura e salto direttamente al punto missione successivo
            gcs().send_text(MAV_SEVERITY_WARNING, "invalid trick");
            printf("\ntrick non riconosciuto\n");
            _complete = true;
            break;
    }
}

/*/funzione booleana per il completamento del trick
bool AC_Trick::trick_completed()
{
    printf("\ntrick completato?\t");
    if(n<3){
        n++;
        printf("no\n");
        return complete;
    }
    printf("si\n");
    n=0;
    complete = true;
    return complete;
}*/

void AC_Trick::do_flip()
{
    printf("\nflip\n");
    
    if(millis() - _start_time_ms > FLIP_TIMEOUT) {
        _state = TrickState::Abandon;
    }

    float throttle_out = _motors.get_throttle(); //prendere il throttle attuale, in motors_class, ci sono due funzioni quale? c'è un get_throttle_in anche in attitude_control
    
    int32_t roll_angle = _ahrs.roll_sensor;
    
    switch (_state) {
        
        case TrickState::Start:
            //under 45 degrees request 400deg/sec roll
            _attitude_control.input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
        
            //increase throttle
            throttle_out += FLIP_THR_INC;
        
            //beyond 45deg lean angle move to next state
            if (roll_angle >= 4500) {
                _state = TrickState::Roll;
            }
        break;
        
        case TrickState::Roll:
            // between 45deg ~ -90deg request 400deg/sec roll
            _attitude_control.input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
            
            // decrease throttle
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

            // beyond -90deg move on to recovery
            if ((roll_angle < 4500) && (roll_angle > -9000)) {      //roll_angle dov'è dichiarato?
                _state = TrickState::Recover;
            }
        break;
        
        case TrickState::Recover:
            // use originally captured earth-frame angle targets to recover
            _attitude_control.input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

            // increase throttle to gain any lost altitude
            throttle_out += FLIP_THR_INC;

            float recovery_angle;
            recovery_angle = fabsf(orig_attitude.x - (float)_ahrs.roll_sensor);

            // check for successful recovery
            if (fabsf(recovery_angle) <= TRICK_RECOVERY_ANGLE) {
                // log successful completion
                //AP::logger().Write_Event(LogEvent::TRICK_END);
                
                _complete = true;
            }
        break;
        
        case TrickState::Abandon:
            gcs().send_text(MAV_SEVERITY_WARNING, "flip not completed");
            
            _attitude_control.input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);
            
            _complete = true;
        break;
    }
    // output pilot's throttle without angle boost
    _attitude_control.set_throttle_out(throttle_out, false, 0.0f);
    
}

void AC_Trick::do_powerloop()
{
    printf("\npowerloop\n");
    
    if(millis() - _start_time_ms > POWERLOOP_TIMEOUT) {
        _state = TrickState::Abandon;
    }
    
    float throttle_out = _motors.get_throttle_out();//prendere il throttle attuale
    
    int32_t pitch_angle = _ahrs.pitch_sensor;
    
    float air_speed;
    
    bool air_speed_est = _ahrs.airspeed_estimate_true(air_speed);
    
    if(!air_speed_est){};
    
    switch (_state) {
    
        case TrickState::Start:
        {
            /*_motors.set_pitch(-0.4f);
            _motors.set_roll(0.0f);
            _motors.set_yaw(0.0f);*/
            _attitude_control.input_euler_angle_roll_pitch_yaw(0.0, -0.7, 0.0, false);
            //throttle increase by 50%
            throttle_out = 0.5f;
            
            if (air_speed > 10.0f) {
                _state = TrickState::Rise;
            }
            break;
        }
        
        case TrickState::Rise:
            //between 0 and 90 degrees
            //pitch increase
            //_attitude_control.input_euler_angle_roll_pitch_yaw(0.0, 40.0, 0.0, false);
            _attitude_control.input_rate_bf_roll_pitch_yaw(0.0, PWRLOOP_ROTATION_RATE, 0.0);
           /* _motors.set_pitch(0.5f);
            _motors.set_roll(0.0f);
            _motors.set_yaw(0.0f);*/
            
            //throttle increase by 50%
            throttle_out = 0.7f;
            
            if (pitch_angle > 8880) {
                _state = TrickState::Turn;
            }
        break;
        
        case TrickState::Turn:
            //_attitude_control.input_euler_angle_roll_pitch_yaw(0.0, 40.0, 0.0, false);
            _attitude_control.input_rate_bf_roll_pitch_yaw( 0.0, PWRLOOP_ROTATION_RATE, 0.0);
            /*_motors.set_pitch(0.5f);
            _motors.set_roll(0.0f);
            _motors.set_yaw(0.0f);*/
            
            //throttle increase by 50%
            throttle_out = 0.7f;
            
            if (pitch_angle < 1000) {
                _state = TrickState::Loop;
            }
        break;

        case TrickState::Loop:
            //between 90 and 350 degrees
            //il pitch deve restare lo stesso
            _attitude_control.input_rate_bf_roll_pitch_yaw(0.0, PWRLOOP_ROTATION_RATE, 0.0);
            /*_motors.set_pitch(0.5f);
            _motors.set_roll(0.0f);
            _motors.set_yaw(0.0f);*/
            //throttle decrese right under hover level
            throttle_out = _thr_hover*0.2f;
            
            if (pitch_angle < -8900) {
                _state = TrickState::Close;
            }
        break;

        case TrickState::Close:
            _attitude_control.input_rate_bf_roll_pitch_yaw(0.0, PWRLOOP_ROTATION_RATE, 0.0);
            /*_motors.set_pitch(0.5f);
            _motors.set_roll(0.0f);
            _motors.set_yaw(0.0f);*/
            throttle_out = 0.5f;

            if (pitch_angle > -1500) {
                _state = TrickState::Recover;
            }
        break;
        
        case TrickState::Recover:
           // use originally captured earth-frame angle targets to recover
            _attitude_control.input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

            // increase throttle to gain any lost altitude
            throttle_out = _thr_hover*1.5f;

            float recovery_angle;
            recovery_angle = fabsf(orig_attitude.x - (float)_ahrs.roll_sensor);

            // check for successful recovery
            if (fabsf(recovery_angle) <= TRICK_RECOVERY_ANGLE) {
                // log successful completion
              //  AP::logger().Write_Event(LogEvent::TRICK_END);
                
                _complete = true;
            }
        break;
        
        case TrickState::Abandon:
            gcs().send_text(MAV_SEVERITY_WARNING, "powerloop not completed");
            
            _attitude_control.input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);
            
            _complete = true;
        break;
    }
    // output pilot's throttle without angle boost
    //_attitude_control.set_throttle_out(throttle_out/*constrain_float(throttle_out, 0.2f, 0.55f)*/, false, 0.0f);
    _motors.set_throttle(throttle_out);
}

void AC_Trick::do_split_s()
{
    printf("\nsplit_s\n");
    
    if(millis() - _start_time_ms > SPLIT_S_TIMEOUT) {
        _state = TrickState::Abandon;
    }
    
    float throttle_out = _motors.get_throttle(); //prendere il throttle attuale, in motors_class, ci sono due funzioni quale? c'è un get_throttle_in anche in attitude_control
    controllo = millis();
    int32_t roll_angle = _ahrs.roll_sensor;
    int32_t pitch_angle = _ahrs.pitch_sensor;
    
    switch (_state) {
        
        case TrickState::Start:
            //under 45 degrees request 400deg/sec roll
            _attitude_control.input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
        
            //increase throttle
            throttle_out += FLIP_THR_INC;
        
            //beyond 45deg lean angle move to next state
            if (roll_angle >= 4500) {
                _state = TrickState::Roll;
            }
        break;
        
        case TrickState::Roll:
            // between 45deg ~ -90deg request 400deg/sec roll
            _attitude_control.input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE, 0.0, 0.0);
            
            // decrease throttle
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

            // beyond -90deg move on to recovery
            if (roll_angle > 7400) {      //roll_angle dov'è dichiarato?
                _state = TrickState::UpSideDown;
                _timer = millis();
            }
        break;

        case TrickState::UpSideDown:
            _attitude_control.input_rate_bf_roll_pitch_yaw(-3*FLIP_ROTATION_RATE, 0.0, 0.0);
            throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);
            
            if((_timer + 310) <= millis()) {
                _state = TrickState::Loop;
                const float angle_max = _aparm.angle_max;
    actual_attitude.x = constrain_float(_ahrs.roll_sensor, -angle_max, angle_max);
    actual_attitude.y = constrain_float(_ahrs.pitch_sensor, -angle_max, angle_max);
    actual_attitude.z = _ahrs.yaw_sensor;
            }
        break;

        case TrickState::Loop:
            //between 90 and 350 degrees
            //il pitch deve restare lo stesso
            _attitude_control.input_rate_bf_roll_pitch_yaw(stabilize(roll_angle, _previous_angle), PWRLOOP_ROTATION_RATE, 0.0);
            /*_motors.set_pitch(0.5f);
            _motors.set_roll(0.0f);
            _motors.set_yaw(0.0f);*/
            //throttle decrese right under hover level
            throttle_out = _thr_hover*0.2f;
            
            if (pitch_angle < -8900) {
                _state = TrickState::Close;
            }
        break;

        case TrickState::Close:
            _attitude_control.input_rate_bf_roll_pitch_yaw(stabilize(roll_angle, _previous_angle), PWRLOOP_ROTATION_RATE, 0.0);
            /*_motors.set_pitch(0.5f);
            _motors.set_roll(0.0f);
            _motors.set_yaw(0.0f);*/
            throttle_out = 0.5f;

            if (pitch_angle > -1500) {
                _state = TrickState::Recover;
            }
        break;
        
        case TrickState::Recover:
            // use originally captured earth-frame angle targets to recover
            _attitude_control.input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

            // increase throttle to gain any lost altitude
            throttle_out = _thr_hover*1.5f;

            float recovery_angle;
            recovery_angle = fabsf(orig_attitude.x - (float)_ahrs.roll_sensor);

            // check for successful recovery
            if (fabsf(recovery_angle) <= TRICK_RECOVERY_ANGLE) {
                // log successful completion
                //AP::logger().Write_Event(LogEvent::TRICK_END);
                
                _complete = true;
            }
        break;
        
        case TrickState::Abandon:
            gcs().send_text(MAV_SEVERITY_WARNING, "split_S not completed");
            
            _attitude_control.input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);
            
            _complete = true;
        break;
        
        case TrickState::Unlimited:
            //_attitude_control.input_rate_bf_roll_pitch_yaw(0.0, 0.0, 0.0);
            //_attitude_control.input_euler_angle_roll_pitch_yaw(-45.0, orig_attitude.y, orig_attitude.z, false);
            throttle_out = _thr_hover*0.2f;
        break;
    }
    
        _previous_angle = _ahrs.roll_sensor;
        _motors.set_throttle(throttle_out);
}
    
    //funzione per stabilizzare l'assetto del roll
float AC_Trick::stabilize(int32_t roll_angle, int32_t previous_angle)
{
    if(roll_angle >= 0 && roll_angle < 17800) {
        if(previous_angle < roll_angle)
            return -20000;
        return 40000;
    }
    else if(roll_angle < 0 && roll_angle > -17800) {
        if(previous_angle > roll_angle)
            return 20000;
        return -40000;
    }
    else {
        if(roll_angle >= 0) {
            if(previous_angle > roll_angle)
                return -10000;
            else if(previous_angle < roll_angle)
                return 10000;
            else
                return 0;
        }
        else {
            if(previous_angle > roll_angle)
                return 10000;
            else if(previous_angle < roll_angle)
                return -10000;
            else
                return 0;
        }
    }
}
