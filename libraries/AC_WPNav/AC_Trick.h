#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>

class AC_Trick
{
public:

    ///costruttore
    AC_Trick(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_AttitudeControl& attitude_control);
    
    ///inizializza la state machine
    void init();
    
    ///prende la figura passata da mavlink e me la salva
    void set_fig_from_cmd(uint16_t trick);
    
    //esegue la mia figura, contiene la state machine
    void do_selected_figure();
    
    //verifica se la figura è stata completata o meno
    bool trick_completed();
    
    static const struct AP_Param::GroupInfo var_info[];
    
protected:
    
    //references to inertial nav and ahrs libraris
    const AP_InertialNav&           _inav;
    const AP_AHRS_View&             _ahrs;
    AC_PosControl&                  _pos_control;
    //const AP_Motors&                _motors;
    AC_AttitudeControl&             _attitude_control;
    
    AP_Int16 _trick;
    uint16_t _trick_;
    uint8_t n=0;
    bool complete;
};
