/*
 RECOVERY MODE.
flys copter above deck and awaits incoming fixed wing aircraft.
attempts to auto-determine that aircraft has arrived and then lower it to the pole.

Hood Technology May 2023
-losh
*/

#include "Copter.h"


#if MODE_RECOVERY_ENABLED == ENABLED
#include "HT_Recovery.h"

/*
 * Init and run calls for AutoRecovery, flight mode
// aRecovery_init - initialise AutoRecovery controller
 */
bool ModeRecovery::init(bool ignore_checks)
{
    //DEBUG
    _takeoff_complete=false;


    hal.console->printf("AutoRECOVERY INIT\n");
    // exit immediately if not enabled.
    if (!g2.ht_recovery.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Recovery not Enabled") ;
        return false;
    }

    //240209 note to self: this can only be here if we use the candidate_mode trick.
    // otherwise, using control_mode gets the wrong thing.
    gcs().send_text(MAV_SEVERITY_WARNING, "H%2.0f>FDL%2.0f>SDL%2.0f>FDUL%2.0f",
                                fabsf((float)g2.ht_recovery.get_hover_talt()),
                                fabsf((float)g2.ht_recovery.get_fastdesc_load_talt()),
                                fabsf((float)g2.ht_recovery.get_slowdesc_load_talt()),
                                fabsf((float)g2.land_alt_low * (0.01f) ) );

    // do common hood auto checks/init.
    if(!ModeHoodAuto::init(ignore_checks)) return false;

    // init local variables.
    _offsetNED.zero() ;
    _pitch_LP = 0;
    _yaw_LP = 0;
    _pause_State = PauseStateEnum::Unpaused;
    _pause_time = (uint32_t) g2.ht_recovery.get_pause_time_sec_param() *1000;
    _takeoff_initiated = false;
    // pre-set next state so that when the copter first starts flying it transistions to climbtoHover.
    _next_state = RecoveryState::TakeOff;
    _state = RecoveryState::OnTheGround;

    //init rope impact checker.
    rope_impact_check(true) ;

    // reset accum pilot nudges
    g2.follow.zero_pilot_offsets();

   // reset time on Quick tab to hover time.
    g2.ht_recovery.set_remain_hover_time( 1000*(uint32_t) g2.ht_recovery.get_hover_time_s() ) ;
    // reset pause time on quick tab of MP.
    update_pause_time();


    return true;
}

bool ModeRecovery::ready_to_arm()
{

    if(!ModeHoodAuto::ready_to_arm()) return false;

    if(is_maritime()){
        gcs().send_text(MAV_SEVERITY_INFO,"maritime");
        if( g2.ht_recovery.puck_offset_enabled() ){
            // save diff vec btn puck and copter at arm.
            if(!get_diff_vec(_starting_offset_puck_NED) ) return false;
        }
    } else {
        _starting_offset_puck_NED.zero();
    }
    return true;
}



//
// AutoLaunch_run -
// runs the state machine for AutoLaunch.
//
void ModeRecovery::run()
{
        if(_takeoff_complete) gcs().send_text(MAV_SEVERITY_ALERT,"taco");

    // if(motors->armed())     hal.console->printf("AR: armed run\n");
        // log output at 10hz
    uint32_t now = AP_HAL::millis();

    // check if logging should occur.
    if ((now - _last_log_ms >= (uint32_t)g2.ht_recovery._logging_pd_ms.get()) || (_last_log_ms == 0)) {
        _log_request = true;
        _last_log_ms = now;
    } else { _log_request = false ;}

    // variables.
    Vector3f desired_velocity_neu_cms; // calc'd vel to pass to guide mode controller
    Vector3f vel_of_target;     // velocity of lead vehicle
    Vector3f dist_vec;          // vector to lead vehicle
    Vector3f dist_vec_offs;     // vector to lead vehicle + offset
    Vector3f req_pilot_vel;     // velocity requested by pilot.

    // get pilot desired climb rate (ie the state of the throttle stick.)
    float pilot_target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    float target_climb_rate = constrain_float(pilot_target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    if(_takeoff_complete) gcs().send_text(MAV_SEVERITY_ALERT,"TCR:%2.1f",target_climb_rate);
    get_target_dist_and_velocity_ned( dist_vec, dist_vec_offs, vel_of_target) ;
    dist_vec -= _starting_offset_puck_NED;
    dist_vec_offs.zero();

    // check if current state is complete.
    check_if_state_complete();

    // State Machine Determination
    _state = get_aRecovery_state( target_climb_rate, _state );

        AP::logger().Write( "AR4",     //log category name.
                            "TimeUS,state,aHst,dz,doz",    //names
                            "s--mm",   //units
                            "F0000",   //scaling
                            "Qbbff",   //format
                            AP_HAL::micros64(),
                            (int8_t) _state,
                            (int8_t) _althold_st,
                            dist_vec.z,
                            dist_vec_offs.z
                             );


    //determine if pilot is trying to pause the auto recovery.
    update_pause_state();

    Vector3f pilot_nudges = g2.follow.get_pilot_offset_nudges();      // retrieve accumulated pilot nudges.

    // Run reqd code dependent on State
    switch (_state) {

        // For the first three cases (motorstopped, groundidel and pre_takeoff), I copied from mode_loiter.
        case RecoveryState::OnTheGround:
            if(_takeoff_initiated){
                check_for_landing();
                return;
            } else {
                run_aRecovery_OnGround( desired_velocity_neu_cms ) ;
                return;
            }
            break;
        case RecoveryState::SpoolDown:
            // relax loiter target if we might be landed
            if (copter.ap.land_complete_maybe) {
                loiter_nav->soften_for_landing();
            }
            return;
            break;
        case RecoveryState::TakeOff:
            //run takeoff logic
            run_TakeOff(desired_velocity_neu_cms,vel_of_target);
            return;
            break ;
        case RecoveryState::GaffHover: FALLTHROUGH;
        case RecoveryState::Hover:
        {
            // collect and low pass INS data for calc offset to pole.
            low_pass_ahrs(RECOV_OLD_LP_FACTOR, RECOV_NEW_LP_FACTOR) ;

            // hover allows pausing and nudging.
            calc_offsets_with_pause(true, dist_vec_offs, dist_vec,pilot_nudges,req_pilot_vel) ;

            FollowModeAccess.calc_des_vel( desired_velocity_neu_cms, dist_vec_offs, vel_of_target, req_pilot_vel ) ;
            break;
        }
        case RecoveryState::BlowinInTheWind:
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0.0f);

            // call z-axis position controller
            pos_control->update_z_controller();

            return;
        case RecoveryState::FastDescentUnLoaded: FALLTHROUGH;
        case RecoveryState::DescentToPole:
            calc_offsets_with_pause(true, dist_vec_offs, dist_vec, pilot_nudges, req_pilot_vel) ;
            // calculate required velocities
            FollowModeAccess.calc_des_vel( desired_velocity_neu_cms, dist_vec_offs, vel_of_target, req_pilot_vel ) ;
            override_Vz_with_pausing( desired_velocity_neu_cms) ;
            break;
        case RecoveryState::ManualLand:
            req_pilot_vel.z = -pilot_target_climb_rate/100; //because down is positive, and update_pilot_nudgez requires m, not cm.
            if( (req_pilot_vel.z > 0.1f) || (req_pilot_vel.z<-0.1f)){
                g2.follow.update_pilot_nudge_z(req_pilot_vel,G_Dt);
            } else {
                req_pilot_vel.z=0;
                g2.follow.zero_pilot_Z_offset();
                _offsetNED.z = -dist_vec.z;
            }

            // update the pilot nudges with requested velocity
            update_pilot_nudge(req_pilot_vel);

            // calc vector with offset, dont add pilot nudge here though, cause it gets added in calc_des_vel
            dist_vec_offs = dist_vec + _offsetNED  ;

            // calc desired velocities to get where we wanna go.
            FollowModeAccess.calc_des_vel( desired_velocity_neu_cms, dist_vec_offs, vel_of_target, req_pilot_vel ) ;
            break;
        default:
            calc_offsets_with_pause(false, dist_vec_offs, dist_vec, pilot_nudges,req_pilot_vel) ;

            // check for un-paused nudging, but only in LAND right now.
            switch(_state){
                case RecoveryState::Land:
                {
                    pilot_nudges = update_pilot_nudge(req_pilot_vel);
                    dist_vec_offs += pilot_nudges ;
                    break;
                }
                default: break;
            }

            // calculate required velocities
            FollowModeAccess.calc_des_vel( desired_velocity_neu_cms, dist_vec_offs, vel_of_target, req_pilot_vel ) ;

            override_Vz_with_pausing( desired_velocity_neu_cms ) ;
    }

    // constrain vel to be inside pilot parameters so were not asking for something crazy by mistake.
    desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -get_pilot_speed_dn(), g.pilot_speed_up);

    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, false, 0, false, 0.0f, false, _log_request);

    // run guided mode controller to translate req vel into roll/pitch and send to attitude controller.
    ModeGuided::run();

    // update target loc bearing/dist for reporting to hud.
    update_wp_dist_bearing();

    if(_log_request || (int8_t)_next_state >0){
        AP::logger().Write( "AR1",     //log category name.
                            "TimeUS,state,dVx,dVy,dVz,vTz,dz,offZ,tCr,dOz,pOz,cVz,aHst",    //names
                            "s-nnnnmmnmmn-",   //units
                            "F0BBBB00B00B0",   //scaling
                            "Qhffffffffffb",   //format
                            AP_HAL::micros64(),
                            (int16_t) _state,
                            desired_velocity_neu_cms.x,
                            desired_velocity_neu_cms.y,
                            desired_velocity_neu_cms.z,
                            vel_of_target.z,
                            dist_vec.z,
                            _offsetNED.z,
                            pilot_target_climb_rate,
                            dist_vec_offs.z,
                            _pause_vectorNED.z,
                            (int8_t) _althold_st
                             );
                AP::logger().Write( "AR2",     //log category name.
                            "TimeUS,state,dPx,dPy,dPz,pnx,pny,pnz,oVz",    //names
                            "s-mmmmmmn",   //units
                            "F0000000B",   //scaling
                            "Qhfffffff",   //format
                            AP_HAL::micros64(),
                            (int16_t) _state,
                            dist_vec_offs.x,
                            dist_vec_offs.y,
                            dist_vec_offs.z,
                            pilot_nudges.x,
                            pilot_nudges.y,
                            pilot_nudges.z,
                            _override_velocity_z_cms_neu
                             );
        }

}

void ModeRecovery::override_Vz_with_pausing( Vector3f &desired_velocity_neu_cms)
{
    //if not paused, override Z-vel request.
    if((_pause_State==PauseStateEnum::NewlyUnpaused)||(_pause_State==PauseStateEnum::Unpaused)) {
        if(_state!=RecoveryState::ReturnToStation){
            if(_vel_change_counter>0){
                desired_velocity_neu_cms.z = cosine_change( _vel_change_counter,VEL_TRANS_COUNT, _old_vel_override_z_cms, _override_velocity_z_cms_neu) ;
                _vel_change_counter-- ;
            } else {
                desired_velocity_neu_cms.z = _override_velocity_z_cms_neu;
            }
        }
    } else {
        desired_velocity_neu_cms.z = 0;
    }
}

void ModeRecovery::calc_offsets_with_pause( bool allow_nudging, Vector3f &dist_vec_offs, Vector3f &dist_vec, Vector3f &pilot_nudges, Vector3f &req_pilot_vel)
{
  switch(_pause_State) {
                case PauseStateEnum::NewlyPaused:
                    if(!calculate_pause_vectorNED(_pause_vectorNED)) gcs().send_text(MAV_SEVERITY_WARNING,"PAUSE PROB") ; //this needs more thought.
                    gcs().send_text(MAV_SEVERITY_WARNING,"PAUSE");
                    g2.follow.zero_pilot_offsets();
                    FALLTHROUGH;
                case PauseStateEnum::Paused:
                {
                    if(allow_nudging){
                        // accum pilot nudge while paused in FDL and SDL.
                        // get pilot nudges from RC/joystick
                        pilot_nudges = update_pilot_nudge(req_pilot_vel);
                    }

                    // add pilot nudges to location.
                    dist_vec_offs = dist_vec + _pause_vectorNED + pilot_nudges;
                    // dist_vec_offs = dist_vec + _pause_vectorNED;
                    break ;
                }
                case PauseStateEnum::NewlyUnpaused:
                    if(allow_nudging) {
                        if(!pilot_nudges.is_zero()){
                            if(calculate_pause_vectorNED(_pause_vectorNED)){
                                _offsetNED.x = _pause_vectorNED.x ;
                                _offsetNED.y = _pause_vectorNED.y ;
                            } else {
                                _offsetNED.x = _pause_vectorNED.x + pilot_nudges.x;
                                _offsetNED.y = _pause_vectorNED.y + pilot_nudges.y;
                            }
                            g2.follow.zero_pilot_offsets();
                        } else { //pilot did not nudge while paused.
                                _offsetNED.x = _pause_vectorNED.x ;
                                _offsetNED.y = _pause_vectorNED.y ;
                        }
                    }
                    gcs().send_text(MAV_SEVERITY_WARNING,"UNPAUSE");
                    FALLTHROUGH;
                default:
                    if(allow_nudging){
                        // accum pilot nudge while paused in FDL and SDL.
                        // get pilot nudges from RC/joystick
                        pilot_nudges = update_pilot_nudge(req_pilot_vel);
                    }
                    dist_vec_offs = dist_vec + _offsetNED + pilot_nudges;
            }

}

//
// get AutoLaunch Mode State
// determine the autoLaunch state from target_climb_Rate
// based on AltHold Get state.
ModeRecovery::RecoveryState ModeRecovery::get_aRecovery_state( float target_climb_rate_cms, ModeRecovery::RecoveryState prevstate)
{

    
    // if(!_takeoff_initiated) {
    //     if (target_climb_rate_cms>(-25)) {
    //         target_climb_rate_cms=g.pilot_speed_up;
    //     }
    // } else { target_climb_rate_cms = -g.pilot_speed_up; }

    if(motors->armed()){
        if (target_climb_rate_cms<(-25)) {
            _althold_st = get_alt_hold_state(target_climb_rate_cms);
        } else {
            target_climb_rate_cms= g.pilot_speed_up;
            _althold_st = get_alt_hold_state( g.pilot_speed_up);
        }
    } else {
        _althold_st = AltHoldModeState::MotorStopped;

    }


        AP::logger().Write( "ARST",     //log category name.
                            "TimeUS,tcr,aHsT,toi,armed,lndCmp,pSt,nSt",    //names
                            "sn------",   //units
                            "FB000000",   //scaling
                            "Qfbbbbbb",   //format
                            AP_HAL::micros64(),
                            target_climb_rate_cms,
                            (int8_t) _althold_st,
                            (int8_t)_takeoff_initiated,
                            (int8_t)motors->armed(),
                            (int8_t)copter.ap.land_complete,
                            (int8_t)prevstate,
                            (int8_t)_next_state
                             );

    switch( _althold_st ){
        case AltHoldModeState::Takeoff:
            switch(_next_state){
                case RecoveryState::TakeOff:
                    switch(prevstate)
                    {
                        case RecoveryState::OnTheGround:
                            start_Takeoff();
                            return RecoveryState::TakeOff ;
                            break;
                        case RecoveryState::Land: FALLTHROUGH;
                        case RecoveryState::SpoolDown:
                            check_for_landing();
                            return prevstate;
                            break;
                        default:
                            return RecoveryState::TakeOff ;
                    }
                    break;
                case RecoveryState::ClimbToHover:
                    start_ClimbToHover();
                    return RecoveryState::ClimbToHover;
                    break;
                default:
                    //dont know how this happens yet.
                    return _next_state;

            }

            break;
        case AltHoldModeState::Flying:
        {
            if(_next_state == RecoveryState::ContinuePrevSt) {
                return prevstate;
            } else {
                _old_vel_override_z_cms = _override_velocity_z_cms_neu;
                _vel_change_counter = VEL_TRANS_COUNT;
                _current_state_time = 0 ;

                //use prevst to hold _next_state, so I can reset _next_state;
                prevstate = _next_state;
                _next_state = RecoveryState::ContinuePrevSt;
                switch(prevstate)
                {
                    case RecoveryState::ManualLand:
                        start_manualLand();
                        break;
                    case RecoveryState::Land:
                        start_Land();
                        break;
                    case RecoveryState::ReCenterOverLanding:
                        start_ReCenterOverLanding();
                        break;
                    case RecoveryState::GaffHover:
                        start_GaffHover();
                        break;
                    case RecoveryState::FastDescentUnLoaded:
                        start_FastDescentUnLoaded();
                        break;
                    case RecoveryState::DescentToPole:
                        start_SlowDescentToPole();
                        break;
                    case RecoveryState::FastDescentLoaded:
                        start_FastDescentLoaded() ;
                        break;
                    case RecoveryState::WaitForWreckingBall:
                        start_WaitForWreckingBall();
                        break;
                    case RecoveryState::ReturnToStation:
                        start_ReturnToStation();
                        break;
                    case RecoveryState::BlowinInTheWind:
                        start_BlowinInTheWind();
                        break;
                    case RecoveryState::Hover:
                        start_Hover();
                        break;
                    case RecoveryState::ClimbToHover:
                        start_ClimbToHover() ;
                        break;
                   default: break;//either RecoveryState::TakeOff or Grounded.
                }
                return prevstate;
            }
            break ;

        }
        default:
            return RecoveryState::OnTheGround;

    }

}

// START_TAKEOFF
// inits takeoff, offsets and min/max descent rates.
void ModeRecovery::start_Takeoff( void )
{
    gcs().send_text(MAV_SEVERITY_WARNING,"TAKEOFF") ;

    if(g2.ht_recovery._takeoff_speed.get() > 0){
        _override_velocity_z_cms_neu = g2.ht_recovery._takeoff_speed.get() ;
    } else {
        _override_velocity_z_cms_neu = g.pilot_speed_up ;
    }

    _offsetNED.zero();
    _offsetNED.z = -10; // the negative is because NED,

    _rc7_button_on_arm = copter.rc7_state_machine();
}

//
// START_ClimbToHover
// moves copter from current location up to dash altitude.
// does not move copter down if you are above altitude.  should it?
//
void ModeRecovery::start_ClimbToHover( void )
{
    _takeoff_initiated = true;

    gcs().send_text(MAV_SEVERITY_WARNING,"CLIMB") ;

    _old_vel_override_z_cms = _override_velocity_z_cms_neu;

    _offsetNED.zero();
    _offsetNED.z = fabsf((float)g2.ht_recovery.get_hover_talt()) * -1;// the negative is because NED,

    _override_velocity_z_cms_neu = g.pilot_speed_up ;

    // calc slow down height. based on x(t=T/2) = v0(t)/2 = v0*T/4
    // T in sec is global dt * vel_trans_count *2
    _slow_down_z_dist = -(g.pilot_speed_up/100) * (G_Dt * 2 * VEL_TRANS_COUNT) / 4 ;

    //calc state duration limit based on override velocity and offsets.
    _current_state_duration_limit = calc_state_dur_limit();
}

void ModeRecovery::start_FastDescentLoaded(void)
{
    // zero out any accumulated nudges that occured during hover.
    g2.follow.zero_pilot_offsets() ;
    _offsetNED.zero();
    _offsetNED.z = fabsf((float)g2.ht_recovery.get_fastdesc_load_talt()) * -1;

    //convert pitch to offset radius.
     //float offset_radius = 7 * (_pitch_LP*0.01) / 15 ;
     float offset_radius = g2.ht_recovery.get_slope() * (_pitch_LP*0.01)  ;

    // add offset based on low-passed lean during hover.
    _offsetNED.x = offset_radius * sinf( DEG_TO_RAD * (_yaw_LP*0.01)  ) ;
    _offsetNED.y = offset_radius * cosf( DEG_TO_RAD * (_yaw_LP*0.01)  ) ;

    gcs().send_text(MAV_SEVERITY_WARNING,"FDL: %2.1f", fabsf(_offsetNED.z)) ;
    _override_velocity_z_cms_neu = (-1) * fabsf( (float) g2.ht_recovery.get_fastdesc_load_vcms());

    _old_vel_override_z_cms = 0;
    _override_velocity_z_cms_neu = constrain_float( _override_velocity_z_cms_neu, -200, 0) ;

    //calc state duration limit based on override velocity and offsets.
    _current_state_duration_limit = calc_state_dur_limit();

}

void ModeRecovery::start_SlowDescentToPole(void)
{
    // ---------------------------------------------
    // reuse _offsetNED.x and _offsetNED.y

    //  gcs().send_text(MAV_SEVERITY_INFO,"toPole: N=%2.1f E=%2.1f", _offsetNED.x, _offsetNED.y) ;

    _offsetNED.z = fabsf((float)g2.ht_recovery.get_slowdesc_load_talt()) * -1;
     gcs().send_text(MAV_SEVERITY_WARNING,"SDL: %2.1f", fabsf(_offsetNED.z)) ;

    _override_velocity_z_cms_neu = (-1) * fabsf( (float) g2.ht_recovery.get_slowdesc_load_vcms());
    _override_velocity_z_cms_neu = constrain_float( _override_velocity_z_cms_neu, -100, 0) ;

    //calc state duration limit based on override velocity and offsets.
    _current_state_duration_limit = calc_state_dur_limit();

}


void ModeRecovery::start_Hover(void)
{
	gcs().send_text(MAV_SEVERITY_WARNING,"HOVER");

    // reset accum pilot nudges
    g2.follow.zero_pilot_offsets();

    _offsetNED.zero();
    _offsetNED.z = fabsf((float)g2.ht_recovery.get_hover_talt()) * -1; // the negative is because NED,

    _current_state_duration_limit = (1000*(uint32_t) g2.ht_recovery.get_hover_time_s() );
}

void ModeRecovery::start_BlowinInTheWind(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Rope Impact Detected") ;

    _current_state_duration_limit = ((uint32_t)g2.ht_recovery._relax_time_ms.get()) ;
}

void ModeRecovery::start_ReturnToStation(void)
{
     // reinit guided mode since we left it for a bit.
    if(!ModeGuided::init(false)){
        gcs().send_text(MAV_SEVERITY_ERROR, "PROB INIT GUID");
    };
    gcs().send_text(MAV_SEVERITY_WARNING,"Return to Station") ;

    _offsetNED.zero();
    _offsetNED.z = fabsf((float)g2.ht_recovery.get_hover_talt()) * -1; // the negative is because NED,

    _current_state_duration_limit = RETURNTOSTATION_TIME_LIMIT_MS ;
}

void ModeRecovery::start_WaitForWreckingBall(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Wait for Wrecking Ball");

    _current_state_duration_limit = (uint32_t)g2.ht_recovery._wreckingball_time_ms.get();
}

void ModeRecovery::start_FastDescentUnLoaded(void)
{
    //remove accumulate pilot offsets to start descending back home.
    g2.follow.zero_pilot_offsets();

    if(g2.ht_recovery._fdul_yaw_p.get() > 0 ) {
        //remember param rate_yaw_p so we can put it back.
        _orig_yawratekP = attitude_control->get_rate_yaw_pid().kP();

        // set the attitude controller to fight yaw rate more in unloaded descent.
        attitude_control->get_rate_yaw_pid().kP( 0.5 ) ;
    }
    // zero out offsets.
    _offsetNED.zero();

    //Depending on mode, calc new offset in Z.
    if( is_maritime()) {
        _offsetNED.z = g2.ht_recovery.get_gaff_talt() * (-1.0f);
    } else {
        _offsetNED.z = (float)g2.land_alt_low * (-0.01f); // the negative is because NED, 0.01 is to convert CM->M
    }
        gcs().send_text(MAV_SEVERITY_WARNING,"FDUL: %2.1f", fabsf(_offsetNED.z)) ;

    // set override vel based on param and constrain.
    _override_velocity_z_cms_neu = (-1) * fabsf( (float) g2.ht_recovery.get_fastdesc_unload_vcms());
    _override_velocity_z_cms_neu = constrain_float( _override_velocity_z_cms_neu, -1.0f*(fabsf((float) get_pilot_speed_dn())), 0) ;

    // calc slow down height. based on x(t=T/2) = v0(t)/2 = v0*T/4
    // T in sec is global dt * vel_trans_count *2
    _slow_down_z_dist = (fabsf(_override_velocity_z_cms_neu)/100) * (G_Dt * 2 * VEL_TRANS_COUNT) / 4 ;

    if(!is_maritime()) {
        // for land based ops, offset copter land point by rtl_offset_m
        _offsetNED.x = g.rtl_offset_m * copter.simple_cos_yaw;
        _offsetNED.y = g.rtl_offset_m * copter.simple_sin_yaw;
    }

    //calc state duration limit based on override velocity and offsets.
    _current_state_duration_limit = calc_state_dur_limit();

}

/// @brief start Gaff Hover
/// @param
void ModeRecovery::start_GaffHover(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING,"GAFF HOVER") ;

    _current_state_duration_limit = (1000 * (uint32_t) g2.ht_recovery.get_gaffhover_time_s() ) ;
}

void ModeRecovery::start_ReCenterOverLanding(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "RECENTERING");
    g2.follow.zero_pilot_offsets();
    _offsetNED.zero();

    // for land based ops, offset copter land point by rtl_offset_m.
    if(!is_maritime()) {
        _offsetNED.x = g.rtl_offset_m * copter.simple_cos_yaw;
        _offsetNED.y = g.rtl_offset_m * copter.simple_sin_yaw;
    }

    _override_velocity_z_cms_neu = 0;
}

/// @brief start land state, set offsets.
/// @paramnone.
void ModeRecovery::start_Land(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING,"LANDING") ;

    _offsetNED.zero();

    // for land based ops, offset copter land point by rtl_offset_m.
    if(!is_maritime()) {
        _offsetNED.x = g.rtl_offset_m * copter.simple_cos_yaw;
        _offsetNED.y = g.rtl_offset_m * copter.simple_sin_yaw;
    }

    // gcs().send_text(MAV_SEVERITY_WARNING, "oX:%2.1f, oY%2.1f", _offsetNED.x, _offsetNED.y);
    _override_velocity_z_cms_neu = (-1) * fabsf((float) g.land_speed );

}

void ModeRecovery::start_manualLand(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING,"PILOT LAND") ;
}



void ModeRecovery::run_aRecovery_OnGround( Vector3f &vel_neu_cms )
{
        attitude_control->reset_rate_controller_I_terms();

        pos_control->set_externally_limited_xy();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to go to zero

        if(is_maritime()) g2.follow.zero_pilot_offsets();

        vel_neu_cms.zero();
}


// UPDATE_PAUSE_STATE
// input USER_PAUSE - state of throttle stick requesting pause or not.
// updates the pause state, and keeps track of how much pause time the user has left.
void ModeRecovery::update_pause_state( void )
{
    if(_state == RecoveryState::OnTheGround) return;

    // determine state of pause button on remote. as of 240317, using the RC7 (1/2 pwr) button
    bool user_pause = (_rc7_button_on_arm != copter.rc7_state_machine()) ;

    static uint32_t lastime = 0;
    uint32_t now = millis();
    uint32_t timedelta = now - lastime;

    if(user_pause) {
        _pause_time = _pause_time - timedelta ;
    } else {
        _current_state_time += timedelta;
    }

    if(user_pause & (_pause_time>0)){
        // user is requesting pause & has pause time.
        switch(_pause_State) {
            case PauseStateEnum::NewlyUnpaused:
                _pause_State=PauseStateEnum::NewlyPaused;
                break ;
            case PauseStateEnum::Unpaused:
                _pause_State=PauseStateEnum::NewlyPaused;
                break;
            default:
             _pause_State=PauseStateEnum::Paused ;
        }
        update_pause_time() ;
    } else {
        //user is requesting resume (or is out of pause time.)
        switch(_pause_State) {
            case PauseStateEnum::NewlyPaused:
                _pause_State=PauseStateEnum::NewlyUnpaused ;
                break ;
            case PauseStateEnum::Paused:
                _pause_State=PauseStateEnum::NewlyUnpaused ;
                break ;
            default:
             _pause_State=PauseStateEnum::Unpaused ;
        }

    }

    lastime = now;
}

bool ModeRecovery::calculate_pause_vectorNED( Vector3f &pauseVector_NED )
{
        // get our location
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
         return false;
    }

    // get target location and velocity
    Location target_loc;
    Vector3f veh_vel;

    if( is_maritime()){
        if (!g2.follow.get_target_location_and_velocity(target_loc, veh_vel)) {
            return false;
        }

        // change to altitude above home if relative altitude is being used
        if (target_loc.relative_alt == 1) {
            current_loc.alt -= AP::ahrs().get_home().alt;
        }

        // calculate offset vector from the puck to the copter.
        pauseVector_NED = target_loc.get_distance_NED(current_loc);
    } else {
        if(!AP::ahrs().get_relative_position_NED_home(pauseVector_NED)) return false;
    }

    return true;
}


void ModeRecovery::check_if_state_complete( void )
{
    float distXY = 0 ;

    Vector3f diffvec ;
    if(!get_V3f_to_current_target_pos(diffvec)) return; //not to the puck but to where we want to be.

    switch (_state) {
        case RecoveryState::TakeOff:
            if(diffvec.z>-8){
                gcs().send_text(MAV_SEVERITY_ALERT,"Dz=%2.3f",diffvec.z);
            }
            if(diffvec.z>=-5) {
                gcs().send_text(MAV_SEVERITY_ALERT,"go CtH");
                _next_state=RecoveryState::ClimbToHover;
            }
            break ;
        case RecoveryState::ClimbToHover:
            if((diffvec.z > (-0.8*fabsf(_slow_down_z_dist))) && (_vel_change_counter<=0)) {
                _next_state = RecoveryState::Hover;
            }
            // if its time to slow down, start the cosine changer.
            if(( diffvec.z > _slow_down_z_dist ) && (_override_velocity_z_cms_neu>5)) {
                _old_vel_override_z_cms = _override_velocity_z_cms_neu;
                _override_velocity_z_cms_neu = 0 ;
                _vel_change_counter = VEL_TRANS_COUNT;
            }

            // check if the wench is stuck and keeping the copter from ascending.
            if(stuck_winch()) _next_state = RecoveryState::Land;

            // if it took too long, somethigns wrong, so land.
            if(state_time_elapsed()) _next_state = RecoveryState::Land;
            break ;
        case RecoveryState::Hover:
            //check if the airplane arrived on the rope.
            if(rope_impact_check()) {
                _next_state = RecoveryState::BlowinInTheWind;
            } else {
                // check if copter is out of hover time or if pilot wants to move on
                if (state_time_elapsed() ){
                    // switch st so copter doesnt drift in althold if this isnt a rope impact st_cmp.
                    _next_state = RecoveryState::FastDescentLoaded;
                } else if (pilot_request_to_move_on()) {
                    // switch st so copter doesnt drift in althold if this isnt a rope impact st_cmp.
                    _next_state = RecoveryState::FastDescentLoaded;
                } else { update_hover_time(); }
            }
            break ;
        case RecoveryState::BlowinInTheWind:
                if( state_time_elapsed()) {
                    _next_state = RecoveryState::ReturnToStation;
                    gcs().send_text(MAV_SEVERITY_WARNING, "RESUME (time)");
                }
            break ;
        case RecoveryState::ReturnToStation:
            if( diffvec.length() < g2.ht_recovery._position_radius_m.get() || state_time_elapsed()){
                _next_state = RecoveryState::WaitForWreckingBall ;
            }
            break ;
        case RecoveryState::WaitForWreckingBall:
            if(state_time_elapsed()) {
                _next_state = RecoveryState::FastDescentLoaded ;
            } else { update_hover_time(); }
            break;
        case RecoveryState::FastDescentLoaded:
            if( ((diffvec.length()) < LOC_RADIUS_M)
                || (diffvec.z<0)
                || state_time_elapsed()
                || ((diffvec.z<_slow_down_z_dist)&&(_vel_change_counter<=0)) ) {
                    _next_state = RecoveryState::DescentToPole;
            }
            if( (diffvec.z < _slow_down_z_dist) && (!is_zero(_override_velocity_z_cms_neu))){
                _old_vel_override_z_cms = _override_velocity_z_cms_neu;
                _override_velocity_z_cms_neu = 0 ;
                _vel_change_counter = VEL_TRANS_COUNT;
            }
            break;
        case RecoveryState::GaffHover:
            if (state_time_elapsed() || pilot_request_to_move_on() ) {
                if(g2.ht_recovery._auto_land) {
                    _next_state = RecoveryState::ReCenterOverLanding;
                } else {
                    _next_state = RecoveryState::ManualLand;
                }
            } else { update_hover_time(); }
            break ;
        case RecoveryState::FastDescentUnLoaded:
                if((diffvec.z < _slow_down_z_dist) && (_override_velocity_z_cms_neu<-10)){
                    _old_vel_override_z_cms = _override_velocity_z_cms_neu;
                    _override_velocity_z_cms_neu = 0 ;
                    _vel_change_counter = VEL_TRANS_COUNT;
                }

                if(((diffvec.z < (0.8f*_slow_down_z_dist)) && (_vel_change_counter<=0)) || state_time_elapsed()) {
                    if(is_maritime()){
                        _next_state = RecoveryState::GaffHover;
                    } else {
                        if(g2.ht_recovery._auto_land){
                            _next_state = RecoveryState::ReCenterOverLanding;
                        } else {
                            _next_state = RecoveryState::ManualLand;
                        }
                    }
                }
            break;
        case RecoveryState::ReCenterOverLanding:
            distXY = safe_sqrt( powf(diffvec.x,2) + powf(diffvec.y, 2)) ;
            if(distXY<3) { _next_state = RecoveryState::Land; }
            break;
        case RecoveryState::DescentToPole:
            //this isnt great. cause you can go past it. need better.
            if( ((diffvec.length()) < LOC_RADIUS_M) || (diffvec.z < 0 ) || state_time_elapsed()) {
                _next_state = RecoveryState::FastDescentUnLoaded;
            }
            break ;
        case RecoveryState::Land:
            // relax loiter target if we might be landed
            if (copter.ap.land_complete_maybe && (fabsf(diffvec.z - _offsetNED.z)<4)) {
                    _next_state = RecoveryState::SpoolDown;
            }
            break;
        case RecoveryState::SpoolDown:
            check_for_landing();
            break;

        default: break;

    }


    if(_log_request) {
    AP::logger().Write( "ARSC",     //log category name.
        "TimeUS,diffVL,state,dx,dy,dz,ovz,vCc,sdD,dXY,nxSt",    //names
        "sm-mmmn-mm-",   //units
        "F00000B0000",   //scaling
        "Qfhffffhffb",   //format
        AP_HAL::micros64(),
        diffvec.length(),
        (int16_t) _state,
        diffvec.x,
        diffvec.y,
        diffvec.z,
        _override_velocity_z_cms_neu,
        (int16_t) _vel_change_counter,
        _slow_down_z_dist,
        distXY,
        (int8_t)_next_state);
    }
}



void ModeRecovery::update_hover_time( void )
{
    int32_t time_remaining=0 ;
    switch(_state ) {
        case RecoveryState::Hover:
            // calculate time remaining till release.
            time_remaining = (1000*(uint32_t)g2.ht_recovery.get_hover_time_s()) - _current_state_time  ;
            break ;
        case RecoveryState::GaffHover:
            time_remaining = ((uint32_t)1000*g2.ht_recovery.get_gaffhover_time_s()) - _current_state_time  ;
            break;
        case RecoveryState::WaitForWreckingBall:
            time_remaining = ((uint32_t)g2.ht_recovery._wreckingball_time_ms.get()) - _current_state_time  ;
        default: ;
    }

    // set state so msg can be delivered to MP.
    if( time_remaining>0) g2.ht_recovery.set_remain_hover_time( (uint32_t) time_remaining ) ;
    else  g2.ht_recovery.set_remain_hover_time( (uint32_t) 0 ) ;

}


void ModeRecovery::update_pause_time( void )
{
    g2.ht_recovery.set_remain_pause_time( (uint32_t) _pause_time ) ;
}


// GET_V3F_TO_CURRENT_TARGET_POS
// get vector to current target position (ie the position you want to be at)
/// @ param current_target_diffvec - vector to return current target vector.
//
bool ModeRecovery::get_V3f_to_current_target_pos(Vector3f &current_target_diffvec)
{
    if(!get_diff_vec(current_target_diffvec)) return false;

    //add in check offsets to difference cause we want to know if we have reached that offset, not the target.
    current_target_diffvec += _offsetNED;
    current_target_diffvec -= _starting_offset_puck_NED;
    return true;
}


void ModeRecovery::update_wp_dist_bearing(void)
{
     Vector3f diffvec ;
    if(get_V3f_to_current_target_pos(diffvec)){

        // calc heading for reporting purposes
        if (is_zero(diffvec.x) && is_zero(diffvec.y)) {
            _bearing_to_target = 0;
            _dist_to_target=0;
        } else {
            // calc dist for reporting purposes.
            _dist_to_target = diffvec.length();
            if(_dist_to_target < 0.025){
                _dist_to_target = 0 ;
                _bearing_to_target = 0;
            } else {
                _bearing_to_target = degrees(atan2f(diffvec.y, diffvec.x));
            }
        }
    } else {
      _bearing_to_target =0;
      _dist_to_target = 0 ;
    }
}


//
// ROPE IMPACT CHECK
// check if airplane arrived by checking if the copter thinks it should change lean angle
// suddenly and dramatically.
//
bool ModeRecovery::rope_impact_check(bool init)
{
    static float prev_targ_lean = -10000;
    if(init) {
        prev_targ_lean = -10000 ;
        return false;
    }

    bool impact = false;
    //calc copter lean from roll/pitch in centideg
    float target_lean = safe_sqrt(powf(pos_control->get_pitch_cd(),2) + 
                                powf(pos_control->get_roll_cd(),2) );

    //these variables are mostly for logging, if logging gets eliminated, they could get eliminated too.
    float lean_LP = 0;
    float targ_lean_dot = 0 ;
    float targ_lean_diff=0;

    // calc lowpassed version of lean for comparison purposes.
    lean_LP = safe_sqrt( _roll_LP*_roll_LP + _pitch_LP*_pitch_LP) ;

    if( prev_targ_lean > -9000){
        // calc dTargLean/dt.
        targ_lean_dot = (target_lean - prev_targ_lean) /G_Dt;
        // calc lean diff with lean_LP.
        targ_lean_diff = (target_lean - lean_LP) ;

        if((targ_lean_diff > g2.ht_recovery._lean_diff_thresh_cd.get()) &&
                (fabsf(targ_lean_dot) > g2.ht_recovery._lean_dot_thresh_cds.get())){
            impact= true ; //after I remove logging, this could just become a return, and get rid of impact boolean.
            }

    }

    if(_log_request){
        AP::logger().Write( "ARRI",     //log category name.
        "TimeUS,LLP,gsp,rlx,Tp,Tr,dTldt,Tldif,TarLean",    //names
        "sdn-ddkdd",   //units
        "FB00BBBBB",   //scaling
        "Qffifffff",   //format
        AP_HAL::micros64(),
        lean_LP,
        ahrs.groundspeed(),
        (int8_t) impact?1:0,
        pos_control->get_pitch_cd(),
        pos_control->get_roll_cd(),
        targ_lean_dot,
         targ_lean_diff,
          target_lean);
    }
        //remember lean for next time.
        prev_targ_lean = target_lean ;

        return impact;
}

bool ModeRecovery::pilot_request_to_move_on(void)
{
    static uint32_t hover_done_req_time = 0 ;

    //check if user says hover is done by holding down throttle for 3s
    if( channel_throttle->get_control_in() <= 10 ){ //oddly, throttle is 0->1000
        if(hover_done_req_time==0) hover_done_req_time = millis() ;
        // if pilot held throttle full down for over 3s, then move on.
        if((millis() - hover_done_req_time )>= MOVEON_LOW_THR_TIMEMS){
            gcs().send_text(MAV_SEVERITY_WARNING,"PILOT OVERRIDE") ;
            return true ;
        }
    } else { hover_done_req_time=0;}

    return false;
}




uint32_t ModeRecovery::calc_state_dur_limit()
{
    Vector3f st_diffvec ;
    if(get_V3f_to_current_target_pos(st_diffvec)) {
        return  (2*1000*(uint32_t)fabsf(100*st_diffvec.z / _override_velocity_z_cms_neu) );
    } else {
        return DEFAULT_STATE_TIME_LIMIT_MS;
    }
}

bool ModeRecovery::stuck_winch()
{
    static float vel_z_LP = 0;
    float rcout_max = 0 ;
    static float rcout_max_LP = 0;
    uint8_t ii=0;

    //collect max pwm rcout.
    for(ii=0;ii<8;ii++){
        if(hal.rcout->read(ii) > rcout_max) rcout_max = hal.rcout->read(ii) ;
    }

    // calculate the lowpassed version of max rcout pwm.
    rcout_max_LP = (WINCH_LP_FACTOR*rcout_max) +((1-WINCH_LP_FACTOR)*rcout_max_LP);

    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)) return false;

    //calculate Lowpass of vert vel.
    vel_z_LP = (WINCH_LP_FACTOR * (fabsf(veh_velocity.z))) + ((1-WINCH_LP_FACTOR) * vel_z_LP) ;

    //logging. DEBUG.
            AP::logger().Write( "ARSW",         //log category name.
            "TimeUS, VZcur,VZ_LP,rcoutLPmax",   //names
            "snnY",   //units
            "F000",   //scaling
            "Qfff",   //format
            AP_HAL::micros64(),
            veh_velocity.z,
            vel_z_LP,
            rcout_max_LP);

    //check for stuck winch.
    if ((vel_z_LP < WINCH_VZ_LIM) && (rcout_max_LP>WINCH_PWM_LIM ) ){
        gcs().send_text(MAV_SEVERITY_WARNING, "WINCH STUCK?") ;
        return true;
    }

    return false;
}

void ModeRecovery::stop(void)
{
    if(g2.ht_recovery._fdul_yaw_p.get() > 0 ) {
        // set the attitude controller to fight yaw rate more in unloaded descent.
        attitude_control->get_rate_yaw_pid().kP(_orig_yawratekP) ;
    }
}


bool ModeRecovery::check_parameters(void)
{
    if(!g2.ht_recovery.check_parameters())
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "Check Recovery Params") ;
        return false;
    }
    return true;
}

void ModeRecovery::run_TakeOff( Vector3f &desired_velocity_neu_cms, Vector3f &targ_veh_vel)
{
float target_climb_rate;
    // if takeoff isnt running yet, check that we are ready to go, then start takeoff.
    if( !takeoff.running()) {

        // start takeoff by calling for motors up.
        takeoff.start(constrain_float(g.pilot_takeoff_alt,TAKEOFF_MIN_ALT,TAKEOFF_MAX_ALT));
    }
// constrain target climb rate with user specified speed.
    target_climb_rate = constrain_float( _override_velocity_z_cms_neu, 0, g.pilot_speed_up);

    // get take-off adjusted pilot and takeoff climb rates
    takeoff.do_pilot_takeoff(target_climb_rate,true); 

            // yes this is the same as the default case, but I dont want to allow pause on takeoff.
            desired_velocity_neu_cms.x = targ_veh_vel.x * 100 ; //convert m/s to cm/s
            desired_velocity_neu_cms.y = targ_veh_vel.y * 100 ; //convert m/s to cm/s
            desired_velocity_neu_cms.z = target_climb_rate;
        _takeoff_complete = !copter.ap.land_complete && !takeoff.running();
        if(_takeoff_complete) gcs().send_text(MAV_SEVERITY_ALERT,"TO comp");
            gcs().send_text(MAV_SEVERITY_ALERT,"rTO");
}

bool ModeRecovery::state_time_elapsed(void)
{
    bool elapsed = (_current_state_time > _current_state_duration_limit) ;
    if(elapsed) gcs().send_text(MAV_SEVERITY_ALERT,"ELAPSED");
    return elapsed;

}
#endif