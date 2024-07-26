#pragma once
#include "HT_AutoModes.h"

#define RECOV_OLD_LP_FACTOR             0.99875f   // LowPass factor for old data
#define RECOV_NEW_LP_FACTOR             0.00125f   // LowPass factor for new data
#define PAUSE_THRESH_FACTOR             0.75     // percent of pilot up speed to be above for pilot to be requesting pause.
#define LAND_VELXY_THRESH               0.5     // m/s, total vel allowed for land based ops
#define RETURNTOSTATION_TIME_LIMIT_MS   15000
#define DEFAULT_STATE_TIME_LIMIT_MS     25000
#define MOVEON_LOW_THR_TIMEMS           3000
#define POLE_TO_WINGSPAN                2.0    // factor of how to approx wingspan based on pole height param.

#define WINCH_LP_FACTOR                 0.005
#define WINCH_PWM_LIM                   1900
#define WINCH_VZ_LIM                    1

#define LOC_RADIUS_M                    1
#define VEL_TRANS_COUNT                 800

//param defaults.
#define DEFAULT_GAFFHOVER_TIME          30
#define DEFAULT_GAFF_TALT               20
#define DEFAULT_HOVER_TIME_S            60
#define DEFAULT_HOVER_TALT              80      //240213: changed to 80 cause cory cant make up his mind.
#define DEFAULT_PAUSE_TIME              100
#define DEFAULT_SLW_DSC_RATE_CMS        (-30)   //240213: changed cause otherwise the 1 sigfig of MP flutters betwee 20 and 30.
#define DEFAULT_FAST_DSC_LOADED_CMS     (-100)
#define DEFAULT_FAST_DSC_UNLD_CMS       (-180)
#define DEFAULT_FDL_TALT                67
#define DEFAULT_SDL_TALT                59
#define DEFAULT_HIGH_CAPT_ALT_CM        2743    //90ft, 240213, cory now says 90ft. 
#define DEFAULT_LOW_CAPT_ALT_CM         1829    //60ft
#define DEFAULT_CAPT_TOL_CM             450
#define DEFAULT_POLE_HGT_CM             396 //13ft
#define DEFAULT_SLOPE_PER_DEG           0.466666
#define DEFAULT_LEAN_DIFF_THRESH_CD     1000 //10deg
#define DEFAULT_LEANDOT_THRESH_CDS      7500 //75deg/sec
#define DEFAULT_RELAX_TIME_MS           1500
#define DEFAULT_LOGGING_PD_MS           10
#define DEFAULT_WRECKINGBALL_TIME_MS    5000
#define DEFAULT_OFFSET_FROM_PUCK_EN     0
#define DEFAULT_TAKEOFF_SPD_CNS         0 //0=use pilot speed up.

// param limits

#define HOVER_TIME_MAX          320     //sec, changed to 500 to accomodate long waits for altius.
#define HOVER_TIME_MIN          20      //sec
#define GAFF_TIME_MAX           120     //sec
#define GAFF_TIME_MIN           10      //sec
#define HOVER_TALT_MAX          135     //m
#define HOVER_TALT_MIN          5       //m
#define POLE_ALT_MAX            1000    //cm
#define POLE_ALT_MIN            20      //cm
#define CAPTURE_ALT_MAX         10000   //cm
#define CAPTURE_ALT_MIN         100     //cm
#define CAPTURE_TOL_MAX         2000    //cm
#define CAPTURE_TOL_MIN         50      //cm
#define GAFF_TALT_MAX           HOVER_TALT_MAX
#define GAFF_TALT_MIN           10      //cm
#define FDL_VCMS_MAX            180     //cm/s
#define FDL_VCMS_MIN            25      //cm/s
#define SDL_VCMS_MAX            100     //cm/s
#define SDL_VCMS_MIN            10      //cm/s
#define FDUL_VCMS_MAX           180     //cm/s
#define FDUL_VCMS_MIN           50      //cm/s
#define SLOPE_MAX               0.8
#define SLOPE_MIN               0.2


class HT_Recovery
{
    public:
    //default constructor.
    HT_Recovery(void) ;

    //do not allow copies.
    HT_Recovery( const HT_Recovery &other ) = delete ;
    HT_Recovery &operator=(const HT_Recovery&) = delete ;

    static HT_Recovery *get_singleton() {
        return _singleton ;
    }

    // parameter list
    static const struct AP_Param::GroupInfo var_info[] ;
    bool        check_parameters(void);

    bool        enabled() const { return (_enabled.get() ==  1); } ;
    bool        puck_offset_enabled() const { return (_offset_from_puck.get() == 1) ; };
    int32_t     get_hover_time_s() const { return _hover_time_s.get() ; } ;
    int8_t      get_pause_time_sec_param() const { return _pause_time_param_sec.get() ; } ;
    int8_t      get_gaffhover_time_s() const{ return _gaffhover_time_sec.get() ; } ;
    int16_t     get_hover_talt() const { return _hover_talt.get() ; } ;
    float       get_slowdesc_load_talt() const  ;
    float       get_fastdesc_load_talt() const ;
    int16_t     get_gaff_talt() const { return _gaff_talt.get() ; } ;
    int16_t     get_capture_height() const ;


    int16_t     get_fastdesc_load_vcms() const { return  _fast_dsc_load_vz.get() ; } ;
    int16_t     get_slowdesc_load_vcms() const { return  _slow_dsc_load_vz.get() ; } ;
    int16_t     get_fastdesc_unload_vcms() const { return _fast_dsc_unload_vz.get() ; } ;

    float       get_slope() const { return _slope_MperDeg.get(); } ;

    void        set_remain_pause_time( uint32_t pause_rem ) ;
    uint32_t    get_remain_pause_time() const { return _pause_time_remain ; } ;
    void        set_remain_hover_time( uint32_t hover_rem ) ;
    uint32_t    get_remain_hover_time() const { return _hover_time_remain ; }

            // relax params
        AP_Int16        _lean_diff_thresh_cd;
        AP_Int16        _lean_dot_thresh_cds;
        AP_Int16        _relax_time_ms;
        AP_Int16        _logging_pd_ms;
        AP_Int16        _wreckingball_time_ms;

        // capture dimensions
        AP_Int16        _capture_high_alt_cm;
        AP_Int16        _capture_low_alt_cm;
        AP_Int16        _capture_alt_toler_cm;
        AP_Int16        _pole_height_cm;

        AP_Int8         _offset_from_puck;
        AP_Int16        _takeoff_speed;
        AP_Float        _position_radius_m;
        AP_Float        _fdul_yaw_p ;
        AP_Int8         _auto_land ;
    protected:
        // Parameters -------------
        AP_Int8         _enabled ;
        AP_Int32        _hover_time_s ;
        AP_Int8         _pause_time_param_sec ;
        AP_Int8         _gaffhover_time_sec ;
            //alts
        AP_Int16        _hover_talt ;
        AP_Int16        _gaff_talt ;
            //vels
        AP_Int16        _fast_dsc_load_vz ;
        AP_Int16        _slow_dsc_load_vz ;
        AP_Int16        _fast_dsc_unload_vz ;
        AP_Float        _slope_MperDeg ;

    private:
        static HT_Recovery *_singleton ;

        // internal variables.
        uint32_t    _pause_time_remain ;
        uint32_t    _hover_time_remain ;


} ;

namespace AP {
    HT_Recovery *ht_recovery() ;
}