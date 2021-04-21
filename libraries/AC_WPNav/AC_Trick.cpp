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
AC_Trick::AC_Trick(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);
    printf("\nchiamo il costruttore\n");
}

//init
void AC_Trick::init()
{
    printf("\ninizializzo la state machine\n");
}

//mi scrivo la figura da eseguire
void AC_Trick::set_fig_from_cmd(uint16_t trick)
{
    _trick_ = trick;
    printf("\nHO RICEVUTO QUESTA FIGURA: %d\n HO SCRITTO QUESTA FIGURA: %d", trick, _trick_);
}

//selettore di figura
void AC_Trick::do_selected_figure()
{
    printf("\neseguo la figura\n");
}

//funzione booleana per il completamento del trick
bool AC_Trick::trick_completed()
{
    printf("\ntrick completato?\t");
    if(n<3){
        n++;
        printf("no\n");
        return false;
    }
    printf("si\n");
    n=0;
    return true;
}
