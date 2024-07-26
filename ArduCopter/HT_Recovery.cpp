#include "Copter.h"
#if MODE_RECOVERY_ENABLED == ENABLED

#include "HT_Recovery.h"



extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo HT_Recovery::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, HT_Recovery, _enabled, 1, AP_PARAM_FLAG_ENABLE),
	AP_GROUPINFO("_HVR_DUR_S",2, HT_Recovery,_hover_time_s, DEFAULT_HOVER_TIME_S),
	AP_GROUPINFO("_HOVER_TALT",3, HT_Recovery,_hover_talt, DEFAULT_HOVER_TALT),
	AP_GROUPINFO("_CAP_ALT_HI",4, HT_Recovery,_capture_high_alt_cm,DEFAULT_HIGH_CAPT_ALT_CM ),
	AP_GROUPINFO("_CAP_TOL_CM",5, HT_Recovery,_capture_alt_toler_cm, DEFAULT_CAPT_TOL_CM ),
	AP_GROUPINFO("_POLE_ALT_CM",6, HT_Recovery,_pole_height_cm, DEFAULT_POLE_HGT_CM ),
	AP_GROUPINFO("_DFL_V_CMS",7, HT_Recovery,_fast_dsc_load_vz, DEFAULT_FAST_DSC_LOADED_CMS),
    AP_GROUPINFO("_DSL_V_CMS",8, HT_Recovery,_slow_dsc_load_vz, DEFAULT_SLW_DSC_RATE_CMS),
    AP_GROUPINFO("_DFUL_V_CMS",9, HT_Recovery,_fast_dsc_unload_vz, DEFAULT_FAST_DSC_UNLD_CMS),
    AP_GROUPINFO("_M_PER_DEG",10, HT_Recovery,_slope_MperDeg, DEFAULT_SLOPE_PER_DEG),
	AP_GROUPINFO("_PAUSE_TIME_S",11, HT_Recovery, _pause_time_param_sec, DEFAULT_PAUSE_TIME),
	AP_GROUPINFO("_GAF_TIME_S",12, HT_Recovery, _gaffhover_time_sec, DEFAULT_GAFFHOVER_TIME ),
	AP_GROUPINFO("_GAF_TALT", 13, HT_Recovery, _gaff_talt, DEFAULT_GAFF_TALT),
	AP_GROUPINFO("_LEAN_DIFF_CD", 14, HT_Recovery, _lean_diff_thresh_cd,DEFAULT_LEAN_DIFF_THRESH_CD ),
	AP_GROUPINFO("_LEAN_DOT_CDS", 15, HT_Recovery, _lean_dot_thresh_cds,DEFAULT_LEANDOT_THRESH_CDS ),
	AP_GROUPINFO("_RLX_TIME_MS", 16, HT_Recovery, _relax_time_ms,DEFAULT_RELAX_TIME_MS ),
	AP_GROUPINFO("_LOG_PD_MS",  17, HT_Recovery, _logging_pd_ms, DEFAULT_LOGGING_PD_MS),
	AP_GROUPINFO("_WRKBALL_MS", 18, HT_Recovery, _wreckingball_time_ms, DEFAULT_WRECKINGBALL_TIME_MS),
	AP_GROUPINFO("_CAP_ALT_LW", 19, HT_Recovery,_capture_low_alt_cm,DEFAULT_LOW_CAPT_ALT_CM ),
    AP_GROUPINFO("_OFF_PUCK_EN",20,HT_Recovery, _offset_from_puck, DEFAULT_OFFSET_FROM_PUCK_EN),
    AP_GROUPINFO("_TKOFF_V_CMS",21, HT_Recovery, _takeoff_speed, DEFAULT_TAKEOFF_SPD_CNS),
    AP_GROUPINFO("_POS_RAD_M", 22, HT_Recovery, _position_radius_m,2),
    AP_GROUPINFO("_FDUL_YAW_P", 23, HT_Recovery, _fdul_yaw_p, 0.4 ),
    AP_GROUPINFO("_AUTO_LAND", 24, HT_Recovery, _auto_land, 0),
	AP_GROUPEND
} ;

HT_Recovery *HT_Recovery::_singleton ;

//DEFAULT CONSTRUCTOR
HT_Recovery::HT_Recovery()
{
    if( _singleton != nullptr ) {
        AP_HAL::panic("HT_Recovery must be singleton") ;
    }
    _singleton = this ;
    AP_Param::setup_object_defaults(this,var_info) ;
}

void HT_Recovery::set_remain_hover_time( uint32_t time)
{
    _hover_time_remain = time ;
}

void HT_Recovery::set_remain_pause_time( uint32_t time)
{
    _pause_time_remain = time ;
}

float HT_Recovery::get_fastdesc_load_talt() const
{
	switch(copter.mode_candidate)
	{
		case Mode::Number::RECOVERY_HIGH:
			return (_hover_talt.get()
                    - 0.01*(_capture_high_alt_cm.get() - _capture_alt_toler_cm.get()
                            - _pole_height_cm.get() - (POLE_TO_WINGSPAN*_pole_height_cm.get())))  ;
		case Mode::Number::RECOVERY_LOW:
			return (_hover_talt.get()
                    - 0.01*(_capture_low_alt_cm.get() - _capture_alt_toler_cm.get()
                            - _pole_height_cm.get() - (POLE_TO_WINGSPAN*_pole_height_cm.get())))  ;
		default:
			return DEFAULT_FDL_TALT ;
	}
}

float HT_Recovery::get_slowdesc_load_talt() const
{
	switch(copter.mode_candidate)
	{
		case Mode::Number::RECOVERY_HIGH:
			return (_hover_talt.get()
                    - 0.01*(_capture_high_alt_cm.get() + _capture_alt_toler_cm.get()
                            - _pole_height_cm.get() - (POLE_TO_WINGSPAN*_pole_height_cm.get())))  ;
		case Mode::Number::RECOVERY_LOW:
			return (_hover_talt.get()
                    - 0.01*(_capture_low_alt_cm.get() + _capture_alt_toler_cm.get()
                            - _pole_height_cm.get() - (POLE_TO_WINGSPAN*_pole_height_cm.get())))  ;
		default:
			return DEFAULT_SDL_TALT ;
	}
}

int16_t HT_Recovery::get_capture_height() const
{
	switch(copter.mode_candidate)
	{
		case Mode::Number::RECOVERY_HIGH:
			return _capture_high_alt_cm.get();
		case Mode::Number::RECOVERY_LOW:
			return _capture_low_alt_cm.get();
		default:
			return 0 ;
	}
}

//
// CHECK_PARAMETERS
// check parameters so copter doesnt attempt something crazy because a user input 60deg dash angles or 1km offsets.
// not that anybody would do that.
bool HT_Recovery::check_parameters(void)
{
    // todo: gaff alt check rel to other alts.

    // CHECK HOVER TIME
    if(get_hover_time_s() > HOVER_TIME_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: Hover Time %li > %is",(long int) get_hover_time_s(), HOVER_TIME_MAX ) ;
        return false;
    }
    if(get_hover_time_s() < HOVER_TIME_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: Hover Time %li < %is", (long int) get_hover_time_s(), HOVER_TIME_MIN ) ;
        return false;
    }
    // CHECK GAFF TIME
    if(get_gaffhover_time_s() > GAFF_TIME_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: Gaff Time %li > %is",(long int) get_gaffhover_time_s(), GAFF_TIME_MAX ) ;
        return false;
    }
    if(get_gaffhover_time_s() < GAFF_TIME_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: GAFF Time %li < %is", (long int) get_gaffhover_time_s(), GAFF_TIME_MIN ) ;
        return false;
    }
    // CHECK HOVER ALT
    if(get_hover_talt() > HOVER_TALT_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: Hover Alt %i > %im", get_hover_talt(), HOVER_TALT_MAX ) ;
        return false;
    }
    if(get_hover_talt() < HOVER_TALT_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: Hover Alt %i < %im", get_hover_talt(), HOVER_TALT_MIN ) ;
        return false;
    }
    // CHECK POLE ALT
    if(_pole_height_cm.get() > POLE_ALT_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: POLE TALT %i > %icm", _pole_height_cm.get(), POLE_ALT_MAX ) ;
        return false;
    }
    if(_pole_height_cm.get() < POLE_ALT_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: POLE ALT %i < %icm", _pole_height_cm.get(), POLE_ALT_MIN ) ;
        return false;
    }
    // CHECK CAPTURE ALT
    if(_capture_high_alt_cm.get() > CAPTURE_ALT_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: CAPT ALT HIGH %i > %icm", _capture_high_alt_cm.get(), CAPTURE_ALT_MAX ) ;
        return false;
    }
    if(_capture_high_alt_cm.get() < CAPTURE_ALT_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: CAPT ALT HIGH %i < %icm", _capture_high_alt_cm.get(), CAPTURE_ALT_MIN ) ;
        return false;
    }
    // CHECK CAPTURE ALT
    if(_capture_low_alt_cm.get() > CAPTURE_ALT_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: CAPT ALT LOW %i > %icm", _capture_low_alt_cm.get(), CAPTURE_ALT_MAX ) ;
        return false;
    }
    if(_capture_low_alt_cm.get() < CAPTURE_ALT_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: CAPT ALT LOW %i < %icm", _capture_low_alt_cm.get(), CAPTURE_ALT_MIN ) ;
        return false;
    }
    // CHECK CAPTURE TOL
    if(_capture_alt_toler_cm.get() > CAPTURE_TOL_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: CAPT TOL %i > %icm", _capture_alt_toler_cm.get(), CAPTURE_TOL_MAX ) ;
        return false;
    }
    if(_capture_alt_toler_cm.get() < CAPTURE_TOL_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: CAPT TOL %i < %icm", _capture_alt_toler_cm.get(), CAPTURE_TOL_MIN ) ;
        return false;
    }
    // CHECK GAFF ALT
    if(get_gaff_talt() > GAFF_TALT_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: GAFF TALT %i > %im", get_gaff_talt(), GAFF_TALT_MAX ) ;
        return false;
    }
    if(get_gaff_talt() < GAFF_TALT_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: GAFF TALT %i < %im", get_gaff_talt(), GAFF_TALT_MIN ) ;
        return false;
    }
    // CHECK FAST DESCENT LOADED VEL
    if( abs(get_fastdesc_load_vcms()) > FDL_VCMS_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: FDL VCMS %i > %icm/s", (int16_t) abs(get_fastdesc_load_vcms()), FDL_VCMS_MAX ) ;
        return false;
    }
    if( abs(get_fastdesc_load_vcms()) < FDL_VCMS_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: FDL VCMS %i < %icm/s", (int16_t) abs(get_fastdesc_load_vcms()), FDL_VCMS_MIN ) ;
        return false;
    }
    // CHECK SLOW DESCENT LOADED VEL
    if(abs(get_slowdesc_load_vcms()) > SDL_VCMS_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: SDL VCMS %i > %icm/s", (int16_t) abs(get_slowdesc_load_vcms()), SDL_VCMS_MAX ) ;
        return false;
    }
    if(abs(get_slowdesc_load_vcms()) < SDL_VCMS_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: SDL VCMS %i < %icm/s", (int16_t)abs(get_slowdesc_load_vcms()), SDL_VCMS_MIN ) ;
        return false;
    }
    // CHECK FAST DESC UNLOADED VEL
    if(abs(get_fastdesc_unload_vcms()) > FDUL_VCMS_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: FDUL VCMS %i > %icm/s", (int16_t)abs(get_fastdesc_unload_vcms()), FDUL_VCMS_MAX ) ;
        return false;
    }
    if(abs(get_fastdesc_unload_vcms()) < FDUL_VCMS_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: FDUL VCMS %i < %icm/s", (int16_t)abs(get_fastdesc_unload_vcms()), FDUL_VCMS_MIN ) ;
        return false;
    }
    // CHECK OFFSET SLOPE
    if(get_slope() > SLOPE_MAX ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: SLOPE %1.2f > %1.2f", get_slope(), SLOPE_MAX ) ;
        return false;
    }
    if(get_slope() < SLOPE_MIN ) {
        gcs().send_text(MAV_SEVERITY_INFO, "AR: SLOPE %1.2f < %1.2f", get_slope(), SLOPE_MIN ) ;
        return false;
    }

    // check alt relative to each other.
    // if(get_fastdesc_load_talt() > get_hover_talt()) {
    //       gcs().send_text(MAV_SEVERITY_INFO, "AR: FDL %i > Hover %im", get_fastdesc_load_talt(), get_hover_talt()) ;
    //     return false;
    // }
    // if(get_slowdesc_load_talt() > get_fastdesc_load_talt()) {
    //       gcs().send_text(MAV_SEVERITY_INFO, "AR: SDL %i > FDL %im", get_slowdesc_load_talt(), get_fastdesc_load_talt()) ;
    //     return false;
    // }

    return true ;
}


namespace AP {
    HT_Recovery *ht_recovery()
    {
        return HT_Recovery::get_singleton() ;
    }
}

#endif