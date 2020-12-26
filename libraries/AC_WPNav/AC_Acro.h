#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library


//altezze minime per le varie figure
#define AC_ACRO_FLIP_MIN_ALT        10000.0f
#define AC_ACRO_POWERLOOP_MIN_ALT   20000.0f
#define AC_ACRO_SPLIT_S_MIN_ALT      20000.0f

#define TRICK_RECOVERY_ANGLE    500
#define FLIP_THR_INC            0.20f   // throttle increase during TrickState::Start stage (under 45deg lean angle)
#define FLIP_THR_DEC            0.24f   // throttle decrease during TrickState::Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE      40000   //rotation rate
#define PWL_THR_INC             0.50f
#define PWL_THR_DEC             0.55f   //verificare
#define SPL_S_THR_INC           0.50f   //verificare
#define AC_ACRO_TRICK_DEFAULT   0

class AC_Acro  
{
public:
    ///constructor
    AC_Acro(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AP_Motors& motors, AC_AttitudeControl& attitude_control, const AP_Vehicle::MultiCopter&  aparm);
    
    ///init
    void init(); //inizializzazione del controller
    
    ///update acro controller
    //bool update() WARN_IF_UNUSED;
    
    ///get desired roll, pitch, yaw different for each figure and state
    //float get_roll() const;
    //float get_pitch() const;
    //float get_yaw() const;
    
    void set_fig_from_cmd(uint16_t cmd_trick) {trick = cmd_trick;}
    //void get_correct_altitude(Vector3f &result, uint16_t trick);
    //float min_altitude(uint16_t trick);
    float get_altitude_error();
    void do_selected_figure();
    bool trick_completed() {return complete;}
    
 //   static const struct AP_Param::GroupInfo var_info[];
    
private:

    Vector3f orig_attitude;
    void do_flip();
    void do_powerloop();
    void do_split_s();
    
    enum SelectFigure : uint16_t {  //enum class invece?
        POWERLOOP   = 0,
        SPLIT_S     = 1,
        FLIP        = 2,
    };
    
    enum class TrickState : uint8_t {
        Start,
        Loop,
        Roll,
        Recover,
    };
    TrickState _state;
    
    //references to inertial nav and ahrs libraris
    const AP_InertialNav&           _inav;
    const AP_AHRS_View&             _ahrs;
    AC_PosControl&                  _pos_control;
    const AP_Motors&                _motors;
    AC_AttitudeControl&             _attitude_control;
    
    //parameters and logger objects
    const AP_Vehicle::MultiCopter         _aparm;
    //AP_Param                        g;
    //const AP_Logger                       _logger;
    
    //GCS_Copter &gcs();
    
    uint16_t trick; //const
    bool complete;
};
