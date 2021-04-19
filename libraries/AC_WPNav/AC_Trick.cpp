#include <AP_HAL/AP_HAL.h>
#include "AC_Trick.h"

extern const AP_HAL::HAL& hal;

//costruttore di default
AC_Trick::AC_Trick(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    //AP_Param::setup_object_defaults(this, var_info);
    printf("\nchiamo il costruttore\n");
}

//init
void AC_Trick::init()
{
    printf("\ninizializzo la state machine\n");
}

void AC_Trick::do_selected_figure()
{
    printf("\neseguo la figura\n");
}

bool AC_Trick::trick_completed()
{
    printf("\ntrick completato?\t");
    if(n<3){
    n++;
    printf("no\n");
    return false;}
    printf("si\n");
    return true;
}
