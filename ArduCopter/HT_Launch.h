#pragma once
#include "HT_AutoModes.h"

#ifndef PI
  #define PI               3.14159265358979f
#endif

#define LAUNCH_OLD_LP_FACTOR           0.995f   // LowPass factor for old data
#define LAUNCH_NEW_LP_FACTOR           0.005f   // LowPass factor for new data
// DASH PARAMS
#define STICK_FULL_FWD          (-ROLL_PITCH_YAW_INPUT_MAX) // full stick fwd in centideg i think
#define AFTER_DASH_RTL_TIME     3000    // msec after release the copter should go to RTL.
#define AFTER_RELEASE_PULLUP_MS 1000    // msec after relase the copter should pull up.
#define MIN_DASH_BEF_UNLOCK_MS  2000    // msec before release copter should unlock.
#define HORZ_OFFSET_INC         1.1f
#define VERT_ASC_MS             2000    // ms. time to ascend straight up in initial ascent before adding offsets.
#define INC_HORZ_VEL_TIME_MS     5000    // ms. time over which to slowly add the horiz offsets to copter for smooth roll out.


// PARAMS FOR INSITU x8 demo.
#define YAW_RATE_LIM            0.5     // rad/s, limit of low-passed yaw rate for allowing release to occur.
#define LEAN_ERROR_LIM          500     // cdeg.  arbitrary 5deg limit.



// parameter constraints
#define DASH_DIR_OFFSET_MAX     90      // deg
#define DASH_TIME_MIN           500     // msec,
#define DASH_TIME_MAX           100000  // msec,
#define DASH_ALT_MIN            20      // m, min alt for dash
#define DASH_ALT_MAX            200     // m, max alt for dash
#define DASH_ANGLE_MIN          500     // currently in centi-deg
#define DASH_ANGLE_MAX          3500    // currently in centi-deg
#define DASH_PREDASH_MIN        500     // ms, minimum time to predash.
#define DASH_PREDASH_MAX        30000   // ms, maximum time to predash.
#define DASH_OFFSET_MIN         0       // m, minimum XY offset from launch point
#define DASH_OFFSET_MAX         200     // m, max XY offset from launch point.

// params for maritime
#define DEBOUNCE_N              100
#define MINIMUM_RC_VAL          4000

class HT_Launch
{
    public:
    //default constuctor.
    HT_Launch(void) ;

    // do not allow copies
    HT_Launch( const HT_Launch &other) = delete;
    HT_Launch &operator=(const HT_Launch&) = delete ;

    static HT_Launch *get_singleton(){
        return _singleton ;
    }

        // parameter list
    static const struct AP_Param::GroupInfo var_info[];

    void        set_remaining_rel_window( uint32_t remaining ) ;
    uint32_t    get_remaining_rel_window( void ) ;
    void        set_lockservo_state( int16_t state ) ;
    int16_t     get_lockservo_state( void ) ;
    void        set_dash_dir( int8_t dashdir ) ;
    int8_t      get_dash_dir(void) ;

    bool        enabled() const { return (_enabled.get() == 1); } ;
    int32_t     get_dash_time_ms() const { return 1000*_dash_time.get() ; } ; //convert sec->msec
    int16_t     get_dash_ang() const { return _dash_angle.get() ; } ;
    int16_t     get_dash_alt_m() const { return _dash_alt_m.get() ; } ;
    int16_t     get_offset() const { return _offset.get() ; } ;
    int16_t     get_predash_ms() const { return _predash_time_ms.get() ; } ;
    bool        autorel() const {return (_auto_release.get()==1) ; } ;
    bool        check_parameters(void);
    int8_t      get_relay_index(void) { return (int8_t)_relay_index ; } ;
    void        toggle_blower( bool onoff) ;
    void        blower_off_check(void);
    void        set_blower_off_time(void);
    void        blower_on(void)   {toggle_blower(true); };
    void        blower_off(void)  {toggle_blower(false) ; };

    int16_t     get_safety_fence_radius_m() {return (int16_t) _safety_fence_radius_m ;} ;

    protected:
        //  Parameters
        AP_Int8     _enabled ;
        AP_Int32    _dash_time;
        AP_Int16    _dash_alt_m;
        AP_Int16    _offset;
        AP_Int16    _predash_time_ms;
        AP_Int16    _dash_angle;
        AP_Int8     _auto_release;
        AP_Int16    _safety_fence_radius_m;
        AP_Int8     _relay_index;

    private:
        static HT_Launch *_singleton;

        // internal variables.
        int16_t     _servo_state;
        uint32_t    _remaining_release_window;
        int8_t      _dash_dir ;
        uint32_t    _blower_off_time;
        bool        _relay_state;

} ;


namespace AP {
    // HT_Launch &ht_launch();
    HT_Launch *ht_launch() ;
};
