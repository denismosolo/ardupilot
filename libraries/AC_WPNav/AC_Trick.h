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
#define FLIP_THR_INC            0.20f   // throttle increase during TrickState::Start stage (under 45deg lean angle)
#define FLIP_THR_DEC            0.24f   // throttle decrease during TrickState::Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE      40000   //rotation rate during flip
#define PWRLOOP_ROTATION_RATE   10000
#define PWL_THR_INC             0.50f
#define PWL_THR_DEC             0.55f
#define FLIP_TIMEOUT             3000    //max time to perform a flip in milliseconds
#define POWERLOOP_TIMEOUT       10000   //max time toperform a powerloop in milliseconds
#define SPLIT_S_TIMEOUT         10000   //max time to perform a split S in milliseconds

class AC_Trick
{
public:

    ///costruttore
    AC_Trick(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_AttitudeControl& attitude_control, AP_Motors& motors, const AP_Vehicle::MultiCopter& aparm);

    ///inizializza la state machine
    void init();

    ///prende la figura passata da mavlink e me la salva
    void set_fig_from_cmd(uint16_t trick);

    //esegue la mia figura, contiene la state machine
    void do_selected_figure();

    //verifica se la figura è stata completata o meno
    bool trick_completed() {return _complete;};

    static const struct AP_Param::GroupInfo var_info[];

protected:

    //references to inertial nav and ahrs libraris
    const AP_InertialNav&           _inav;
    const AP_AHRS_View&             _ahrs;
    AC_PosControl&                  _pos_control;
    AC_AttitudeControl&             _attitude_control;
    AP_Motors&                      _motors;
    const AP_Vehicle::MultiCopter   _aparm;

    AP_Int16 _trick;
    //uint16_t _trick_;
    uint8_t n=0;

private:

    enum SelectTrick : uint16_t {   //non uso enum class perché altrimenti dovrei definire _trick di tipo SelectTrick ma è già di tipo AP_Int16 in quanto parametro
        POWERLOOP = 0,
        SPLIT_S = 1,
        FLIP = 2,
    };

    enum TrickState : uint16_t {    //non uso enum class perché dovrei definire nello switch un caso per ogni voce sempre
        Start,
        Rise,
        Turn,
        Roll,
        Recover,
        Loop,
        Close,
        UpSideDown,
        Abandon,
        Unlimited,
    };

    void do_flip();
    void do_powerloop();
    void do_split_s();
    float stabilize(int32_t roll_angle, int32_t previous_angle);

    Vector3f orig_attitude, actual_attitude;
    float _thr_hover;
    uint16_t _state;
    uint32_t _start_time_ms;
    uint32_t _timer;
    uint32_t controllo;
    int32_t _previous_angle;
    bool _complete;
};

using AP_HAL::millis;
