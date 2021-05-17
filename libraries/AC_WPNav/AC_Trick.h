#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <GCS_MAVLink/GCS.h>

#define TRICK_RECOVERY_ANGLE    500
#define FLIP_THR_INC            0.20f       //throttle increase during TrickState::Start stage (under 45deg lean angle)
#define FLIP_THR_DEC            0.24f       //throttle decrease during TrickState::Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE      40000       //rotation rate during flips in deg/s
#define LOOP_ROTATION_RATE      10000       //rotation rate during loops in deg/s
#define FLIP_MIN_ALT            1000        //min altitude to perform a flip in cm
#define PWRLOOP_MIN_ALT         6000        //min altitude to perform a powerloop in cm
#define SPLIT_S_MIN_ALT         3500        //min altitude to perform a splitS in cm
#define FLIP_TIMEOUT            3000        //max time to perform a flip in milliseconds
#define POWERLOOP_TIMEOUT       60000       //max time toperform a powerloop in milliseconds
#define SPLIT_S_TIMEOUT         7000        //max time to perform a split S in milliseconds

class AC_Trick
{
public:

    ///costruttore
    AC_Trick(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_AttitudeControl& attitude_control, AP_Motors& motors, const AP_Vehicle::MultiCopter& aparm);

    ///inizializza la state machine
    void init();

    ///prende la figura passata da mavlink e me la salva
    void set_fig_from_cmd(uint16_t trick);

    //modifico se necessario la quota ricevuta da mavlink
    bool altitude_check(int32_t* quota);
    
    //esegue la mia figura, contiene la state machine
    void do_selected_figure();

    //verifica se la figura è stata completata o meno
    bool trick_completed() {return _complete;};

    static const struct AP_Param::GroupInfo var_info[];

protected:

    //riferimenti alle librerie utilizzate
    const AP_InertialNav&           _inav;
    const AP_AHRS_View&             _ahrs;
    AC_PosControl&                  _pos_control;
    AC_AttitudeControl&             _attitude_control;
    AP_Motors&                      _motors;
    const AP_Vehicle::MultiCopter   _aparm;
    
    //contiene la figura da eseguire
    AP_Int16 _trick;

private:

    enum SelectTrick : uint16_t {
        POWERLOOP = 0,
        SPLIT_S = 1,
        FLIP = 2,
        ROTATE =3,
    };

    enum TrickState : uint16_t {
        Start,
        Rise,
        Roll,
        Recover,
        Loop,
        UpSideDown,
        Abandon,
    };

    void do_flip();
    void do_powerloop();
    void do_split_s();
    void do_rotation();
    float stabilize(int32_t roll_angle, int32_t previous_angle);

    Vector3f orig_attitude;
    float _thr_hover;
    uint16_t _state;
    uint32_t _start_time_ms;
    uint32_t _timer;
    int32_t _previous_angle;
    bool _complete;
};

using AP_HAL::millis;
