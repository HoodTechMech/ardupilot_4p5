#include "HT_Launch.h"
#include "Copter.h"

extern const AP_HAL::HAL& hal;

#define DEFAULT_DASH_DIR_OFFSET 0      // default auto dash direction is to dash in direct of copter at arming
#define DEFAULT_AL_DASH_ANGLE   1700        // default auto-dash angle is 17deg.
#define DEFAULT_AUTO_RELEASE    0
#define DEFAULT_DASH_TIME       30        // default auto-dash dash length is 25s
#define DEFAULT_DASH_ALT        152          // default auto-dash altitude is 152m=500ft
#define DEFAULT_OFFSET_M        100
#define DEFAULT_SLEW_MS         1000
#define DEFAULT_PREDASH_MS      4000
#define DEFAULT_SAFE_FENCE_R    50
#define DEFAULT_BLOWER_RLY_IDX  0
#define BLOWER_ON_GRACE_PD      30000   //msec

const AP_Param::GroupInfo HT_Launch::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, HT_Launch, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: DASH_DUR_SEC
	// @DisplayName: Dash Time
	// @Description: Length of auto dash in milliseconds
	// @Units: Milliseconds
	// @Range: 1000 60000
	// @User: Advanced
	AP_GROUPINFO("_DASH_DUR_SEC",2, HT_Launch,_dash_time, DEFAULT_DASH_TIME),

    // @Param: DASH_ALTITUDE
	// @DisplayName: Dash Altitude
	// @Description: Altitude the copter will climb to before dashing in autolaunch mode.
	// @Units: meters
	// @Range: 2000 to 50000
	// @User: Advanced
	AP_GROUPINFO("_ALT_M", 3, HT_Launch, _dash_alt_m, DEFAULT_DASH_ALT),

    // @Param: OFFSET
	// @DisplayName:
	// @Description: Altitude the copter will climb to before dashing in autolaunch mode.
	// @Units: meters
	// @Range: 2000 to 50000
	// @User: Advanced
    AP_GROUPINFO("_OFFSET_M", 4, HT_Launch, _offset, DEFAULT_OFFSET_M),

    AP_GROUPINFO("_PREDASH_MS",5, HT_Launch,_predash_time_ms, DEFAULT_PREDASH_MS),

    AP_GROUPINFO("_DASH_ANG_CD", 6, HT_Launch,_dash_angle, DEFAULT_AL_DASH_ANGLE) ,

    AP_GROUPINFO("_AUTO_REL",7, HT_Launch,_auto_release, DEFAULT_AUTO_RELEASE),

    AP_GROUPINFO("_SAFE_RAD_M",8, HT_Launch,_safety_fence_radius_m, DEFAULT_SAFE_FENCE_R),

    AP_GROUPINFO("_BLW_RLY_IDX",9, HT_Launch,_relay_index, DEFAULT_BLOWER_RLY_IDX),

    // @Param: DASH_DIRECTION_OFFSET
	// @DisplayName: Dash Direction
	// @Description: direction to dash, degrees, -1 = direction at arming.
	// @Units: Milliseconds
	// @Range: -1 to 359
	// @User: Advanced
	//AP_GROUPINFO("_DIR_OFF", 3, AL_dash_direction_offset,DEFAULT_DASH_DIR_OFFSET),

    AP_GROUPEND
} ;

HT_Launch *HT_Launch::_singleton;



// Default Constructor.
HT_Launch::HT_Launch( )
{
    if( _singleton != nullptr ) {
        AP_HAL::panic("HT_Launch must be singleton") ;
    }
    _singleton = this;
    _relay_state=false;

    AP_Param::setup_object_defaults(this, var_info) ;
}

void HT_Launch::set_remaining_rel_window( uint32_t remaining )
{
    if(remaining > 0 ){
        _remaining_release_window = remaining;
    }
}

uint32_t HT_Launch::get_remaining_rel_window( void )
{
    return _remaining_release_window ;
}

void HT_Launch::set_lockservo_state( int16_t state)
{
    if(state>=0) {
        _servo_state = state;
    }
}

int16_t HT_Launch::get_lockservo_state()
{
    return _servo_state;
}

void HT_Launch::set_dash_dir( int8_t dashdir )
{
    if(dashdir>=0){ _dash_dir = dashdir ; }
}

int8_t HT_Launch::get_dash_dir(void)
{
    return _dash_dir ;
}

//
// CHECK_PARAMETERS
// check parameters so copter doesnt attempt something crazy because a user input 60deg dash angles or 1km offsets.
// not that anybody would do that.
bool HT_Launch::check_parameters(void)
{
    // constrain input params.
    if (get_dash_time_ms() > DASH_TIME_MAX){
        gcs().send_text(MAV_SEVERITY_WARNING, "DashTime %li > %ims",(long int) get_dash_time_ms(),DASH_TIME_MAX  ) ;
        return false;
    }
    if (get_dash_time_ms() < DASH_TIME_MIN){
        gcs().send_text(MAV_SEVERITY_WARNING, "DashTime %li < %ims",(long int) get_dash_time_ms(),DASH_TIME_MIN ) ;
        return false;
    }
    // if (fabsf(AL_dash_direction_offset.get()) > DASH_DIR_OFFSET_MAX){
    //     gcs().send_text(MAV_SEVERITY_WARNING, "DirOff |%li| > %i",(long int) AL_dash_direction_offset.get(), DASH_DIR_OFFSET_MAX ) ;
    //     return false;
    // }
    if (get_dash_alt_m() > DASH_ALT_MAX ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DashAlt %li > %icm",(long int) get_dash_alt_m(), DASH_ALT_MAX ) ;
        return false;
    }
    if (get_dash_alt_m() < DASH_ALT_MIN ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DashAlt %li < %icm",(long int) get_dash_alt_m(), DASH_ALT_MIN ) ;
        return false;
    }
    if (get_dash_ang() > DASH_ANGLE_MAX ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DashAng %li > %icdeg",(long int) get_dash_ang(), DASH_ANGLE_MAX ) ;
        return false;
    }
    if (get_dash_ang() < DASH_ANGLE_MIN ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DashAng %li < %icdeg",(long int) get_dash_ang(), DASH_ANGLE_MIN ) ;
        return false;
    }
    if (get_predash_ms() > DASH_PREDASH_MAX ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PreDashMs %li > %ims",(long int) get_predash_ms(), DASH_PREDASH_MAX ) ;
        return false;
    }
    if (get_predash_ms() < DASH_PREDASH_MIN ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PreDashMs %li < %ims",(long int) get_predash_ms(), DASH_PREDASH_MIN ) ;
        return false;
    }
    if (get_offset() > DASH_OFFSET_MAX ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Offset %li > %im",(long int) get_offset(), DASH_OFFSET_MAX ) ;
        return false ;
    }
    if (get_offset() < DASH_OFFSET_MIN ) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Offset %li < %im",(long int) get_offset(), DASH_OFFSET_MIN ) ;
        return false ;
    }
    return true ;
}

void HT_Launch::toggle_blower( bool onoff)
{
    // reset blower off time just to make sure we dont turn it off incorrectly.
    _blower_off_time = 0 ;

    if(get_relay_index()>-1){
        AP_ServoRelayEvents *sre = AP::servorelayevents();
        if (sre != nullptr) {
            if(onoff){
                if(sre->do_set_relay(get_relay_index(), 1)){
                    gcs().send_text(MAV_SEVERITY_WARNING, "BLOWER ON");
                    _relay_state = true;
                } else { gcs().send_text(MAV_SEVERITY_ERROR,"BLOWER[%d] NOT ON",get_relay_index()); }
            } else {
                if(sre->do_set_relay(get_relay_index(), 0)){
                    gcs().send_text(MAV_SEVERITY_WARNING, "BLOWER OFF");
                    _relay_state = false;
                }
            }
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR,"BLOWER ACCESS ERROR");
        }
    }
}

void HT_Launch::blower_off_check()
{
    if( _blower_off_time > 0){
        if((millis() - _blower_off_time)> BLOWER_ON_GRACE_PD){
            toggle_blower(false);
        }
    }
}

void HT_Launch::set_blower_off_time(void)
{
    if(_relay_state) {
        _blower_off_time = millis() ;
        gcs().send_text(MAV_SEVERITY_WARNING,"%dsec to BLOWER OFF", (int16_t)((float)BLOWER_ON_GRACE_PD/1000));
    }
};
namespace AP {
    HT_Launch *ht_launch()
    {
        return HT_Launch::get_singleton();
    }
}