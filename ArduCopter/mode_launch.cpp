/*
AUTO LAUNCH MODE.
flys copter in auto-dash at DASH_ANGLE for set amount of time
before releasing airplane and then coming home.

Hood Technology May 2023
-losh
*/

#include "Copter.h"
#if MODE_LAUNCH_ENABLED==ENABLED
#include <AP_GPS/AP_GPS.h>
#include "HT_Launch.h"

/*
 * Init and run calls for AutoLaunch, flight mode
// aLaunch_init - initialise AutoLaunch controller
 */
bool ModeLaunch::init(bool ignore_checks)
{
    hal.console->printf("Launch INIT\n");
    // exit immediately if not enabled.
    if (!g2.ht_launch.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Launch not Enabled") ;
        return false;
    }

    // do common hood auto checks.
    if(!ModeHoodAuto::init(ignore_checks)) return false;

    // servo initialization.
    if(!init_servos()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Servo Init Fail");
        return false ;
    }

    if(_release_servo != nullptr) {
        _release_servo->get_servo_safe();
    }

    g2.ht_launch.set_remaining_rel_window((uint32_t)g2.ht_launch.get_dash_time_ms() - MIN_DASH_BEF_UNLOCK_MS ) ;
    g2.ht_launch.set_dash_dir( (int8_t) MaritimeDashDir::NotAssigned ) ;

    // initialize the dash direction
    //init_dash_direction(g2.ht_launch.AL_dash_direction_offset.get());
    init_dash_direction( 0 );

    if( g2.ht_launch.autorel() == 0) { gcs().send_text(MAV_SEVERITY_WARNING, "Manual Release") ; }
    else { gcs().send_text(MAV_SEVERITY_WARNING, "AUTO RELEASE" ) ; }

    // init local variables.
    _state_start_time = 0 ;
    _yaw_rate_LP = 0;
    _roll_LP = 0 ;
    _pitch_LP = 0;
    _ascent_rate= 0.0f;
    _released = false;

    _aLaunch_state = AutoLaunchState::aLaunch_Takeoff;
    _state_complete = true;
    _dash_dir = MaritimeDashDir::NotAssigned ;
    _ALoffsetNED.zero();
    _last_dash_dir= MaritimeDashDir::NotAssigned;

    return true;
}

bool ModeLaunch::ready_to_arm()
{
       if(!ModeHoodAuto::ready_to_arm()) return false;

       if (_dash_dir == MaritimeDashDir::NotAssigned) {
            gcs().send_text(MAV_SEVERITY_WARNING, "NO DASH DIR");
            return false ;
        }

    return true;
}

//
// AutoLaunch_run -
// runs the state machine for AutoLaunch.
//
void ModeLaunch::run()
{
  //  if(_start_burping) gcs().send_text(MAV_SEVERITY_WARNING, "run");

    // standard autolaunch variables.
    float target_roll = 0 ;
    float target_pitch = 0 ;

    // Maritime variables.
    Vector3f desired_velocity_neu_cms; // calc'd vel to pass to guide mode controller
    Vector3f vel_of_target;     // velocity of lead vehicle
    Vector3f dist_vec;          // vector to lead vehicle
    Vector3f temp_dist;
    Vector3f dist_vec_offs;     // vector to lead vehicle + offset
    Vector3f req_pilot_vel;     // velocity requested by pilot.

    // get pilot desired climb rate (ie the state of the throttle stick.)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // State Machine Determination
    _aLaunch_state = get_aLaunch_state( target_climb_rate, _aLaunch_state );

    // calc length of time in particular state.
    _state_time = millis() - _state_start_time ;

    //local variable to hold which controller to run attitude or guided-velocity
    ControllerType controller_in_charge=ControllerType::AttitudeController;

    // Run reqd code dependent on State
    switch (_aLaunch_state) {

        // For the first three cases (motorstopped, groundidel and pre_takeoff), I copied from mode_loiter.
        case AutoLaunchState::aLaunch_OnTheGround:
            run_OnGround( target_roll, target_pitch, target_climb_rate) ;
            break;

        case AutoLaunchState::aLaunch_Takeoff:
            get_target_dist_and_velocity_ned(dist_vec, dist_vec_offs, vel_of_target) ;
            temp_dist = dist_vec;//debug logging.
            run_Takeoff( desired_velocity_neu_cms, vel_of_target,dist_vec );
            controller_in_charge=ControllerType::GuidedVelController;
            break ;

        case AutoLaunchState::aLaunch_InitAsc:
            get_target_dist_and_velocity_ned(dist_vec, dist_vec_offs, vel_of_target) ;
            temp_dist = dist_vec;//debug logging.

            //note this is slight diff than expected: we are not passing the dist_vec_offs vector b/c it already has the follow mode offset incorporated.
            // instead we are passing the dist_vec which is difference in location between copter and target, we'll add our own offsets later.
            controller_in_charge=ControllerType::GuidedVelController;
            run_InitAsc(desired_velocity_neu_cms,vel_of_target, dist_vec, req_pilot_vel ) ;
            break ;
        case AutoLaunchState::aLaunch_SlowToDashAlt:
            get_target_dist_and_velocity_ned(dist_vec, dist_vec_offs, vel_of_target) ;
            temp_dist = dist_vec; //debug logging.

            //note this is slight diff than expected: we are not passing the dist_vec_offs vector b/c it already has the follow mode offset incorporated.
            // instead we are passing the dist_vec which is difference in location between copter and target, we'll add our own offsets later.
            controller_in_charge=ControllerType::GuidedVelController;
            run_SlowToDashAlt(desired_velocity_neu_cms,vel_of_target, dist_vec, req_pilot_vel ) ;
            break ;
        case AutoLaunchState::aLaunch_PreDash:
            get_target_dist_and_velocity_ned(dist_vec, dist_vec_offs, vel_of_target) ;
            temp_dist = dist_vec;

            // set controller to hybrid vel and angle controller.
            controller_in_charge=ControllerType::PreDashController;
            run_PreDash( target_roll, target_pitch, desired_velocity_neu_cms ) ;
            break ;

        case AutoLaunchState::aLaunch_Dashing:
            run_Dash( target_roll, target_pitch ) ;
            break ;

        case AutoLaunchState::aLaunch_Release:
            run_Release( target_roll, target_pitch ) ;
            break ;

        case AutoLaunchState::aLaunch_PostDash:
            run_PostDash( target_roll, target_pitch ) ;
            break ;
    }

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - _last_log_ms >= LOG_PD) || (_last_log_ms == 0)) {
        log_request = true;
        _last_log_ms = now;
    }

    switch( controller_in_charge ) {
        case ControllerType::AttitudeController:
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 0.0f);

            // call z-axis position controller
            pos_control->update_z_controller();
            break ;

        case ControllerType::GuidedVelController:
            // re-use guided mode's velocity controller (takes NEU)
            ModeGuided::set_velocity(desired_velocity_neu_cms, false, 0, false, 0.0f, false, log_request);

            // run guided mode controller to translate req vel into roll/pitch and send to attitude controller.
            ModeGuided::run();
            break ;

        case ControllerType::PreDashController:

            // update position controller with new target
            pos_control->set_vel_desired_cms(desired_velocity_neu_cms);

            // use requested angles for horiz plane.
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 0.0f);

            // run pos controller for Z control.
            pos_control->update_z_controller();

            break;
    }

    // collect and low pass INS data for checking orientation before release.
    low_pass_ahrs(LAUNCH_OLD_LP_FACTOR,LAUNCH_NEW_LP_FACTOR) ;

    if(log_request) {
        //DEBUG Logging for now.  this is being logged at 333Hz so thats nuts.  Need to reduce params and freq.
        // EVENTUALLY, this logging should be at 10hz per above.
        //
        AP::logger().Write( "AULA",     //log category name.
                            "TimeUS,state,Troll,Tpitch,DF,dVz,PitLP,dx,dy,dz,dVx,dVy,altHst",    //names
                            "s-dd-ndmmmnn-",   //units
                            "F0BB0BB000BB0",   //scaling
                            "Qhffffffffffh",   //format
                            AP_HAL::micros64(),
                            (int16_t) _aLaunch_state,
                            target_roll,
                            target_pitch,
                            _dash_factor,
                            desired_velocity_neu_cms.z,
                            get_LP_pitch(),
                            temp_dist.x,temp_dist.y, temp_dist.z,
                            desired_velocity_neu_cms.x, desired_velocity_neu_cms.y,
                            (int16_t) _logging_althold_st);
    }
}

//
// get AutoLaunch Mode State
// determine the autoLaunch state from target_climb_Rate
// based on AltHold Get state.
ModeLaunch::AutoLaunchState ModeLaunch::get_aLaunch_state( float target_climb_rate_cms, ModeLaunch::AutoLaunchState prevstate)
{
    AltHoldModeState althold_state ;
    if (target_climb_rate_cms<(-25)) {
        althold_state = get_alt_hold_state(target_climb_rate_cms);
    } else {
        target_climb_rate_cms= g.pilot_speed_up;
        althold_state = get_alt_hold_state( g.pilot_speed_up);
    }

    //DEBUG
    _logging_althold_st = althold_state;

    static uint8_t counter=0;
    if(counter>200) { counter=0;} else { counter++;}

    switch( althold_state ){
        case AltHoldModeState::Takeoff:
            return AutoLaunchState::aLaunch_Takeoff;
            break;
        case AltHoldModeState::Flying:
        {
            if(_state_complete)
            {
                _state_start_time =  millis() ;
                _state_complete = false;
                switch(prevstate)
                {
                    case AutoLaunchState::aLaunch_PostDash:
                        send_copter_home();
                        return prevstate;
                    case AutoLaunchState::aLaunch_Release:
                        start_PostDash();
                        return AutoLaunchState::aLaunch_PostDash;
                    case AutoLaunchState::aLaunch_Dashing:
                        return AutoLaunchState::aLaunch_Release;
                    case AutoLaunchState::aLaunch_PreDash:
                        start_Dash();
                        return AutoLaunchState::aLaunch_Dashing;
                    case AutoLaunchState::aLaunch_InitAsc:
                        start_SlowToDashAlt();
                        return AutoLaunchState::aLaunch_SlowToDashAlt;
                    case AutoLaunchState::aLaunch_SlowToDashAlt:
                        start_PreDash();
                        return AutoLaunchState::aLaunch_PreDash ;
                    case AutoLaunchState::aLaunch_OnTheGround:
                        if(target_climb_rate_cms>0){
                            gcs().send_text(MAV_SEVERITY_WARNING,"Fly without TO?") ;
                            return AutoLaunchState::aLaunch_Takeoff;
                        } else { _state_complete = true;
                        return AutoLaunchState::aLaunch_OnTheGround;
                        }
                        break;
                   default: //either AutoLaunchState::aLaunch_Takeoff or Grounded.
                        start_InitAsc();
                        return AutoLaunchState::aLaunch_InitAsc;
                }
            } else { return prevstate; }
            break ;

        }
        default:    return AutoLaunchState::aLaunch_OnTheGround;

    }

}

//
// Attempt_TO_AutoLaunch
// send servo cmdto release airplane if conditions are right.
bool ModeLaunch::attempt_to_release( void )
{
    if(allow_release()){
        if(_release_servo != nullptr ) {
            // release and check if msg should be displayed.
            if(_release_servo->set_servo_unsafe() & !_release_alert){
                    gcs().send_text(MAV_SEVERITY_WARNING,"RELEASE RELEASE");
                    _release_alert=true;
                    _released = true;
            }
        }
    }
    return _release_alert;
}


//
// SEND_COPTER_HOME
// chooses correct mode (RTL or FOLLOW) based on configuration at end of mission.
//
bool ModeLaunch::send_copter_home(void)
{
    if( _release_servo != nullptr ) {
        // if you going home, put release servo back where it belongs.
        _release_servo->set_servo_safe();
    }

    if( _lock_servo != nullptr ) {
        // if going home, lock the lock-unlock servo so no indavertent funny business occurs.
        _lock_servo->set_servo_safe() ;
    }

    return ModeHoodAuto::send_copter_home() ;

}


// CALC_ANGLES_FOR_DASH
// calculates roll and pitch and climb rates for current needs.
// fDASH_FACTOR [0,1]: how much of full throttle to use
// bASCENDING: whether or not the copter should ascend.
// fTARGET_ROLL: returned target roll
// fTARGET_PITCH: returned target pitch.
//
void ModeLaunch::calc_angles_for_dash( float dash_factor, float ascent_rate, float &target_roll, float &target_pitch)
{
    // (for right now?) dont let pilot ask for increase in altitude.
    pos_control->set_pos_target_z_from_climb_rate_cm(ascent_rate);

    // Update "simple mode" as if user is pressing down on stick in desired direction.
    channel_roll->set_control_in(  _dash_EWlean*ahrs.cos_yaw()*dash_factor + _dash_NSlean*ahrs.sin_yaw()*dash_factor);
    channel_pitch->set_control_in(-_dash_EWlean*ahrs.sin_yaw()*dash_factor + _dash_NSlean*ahrs.cos_yaw()*dash_factor);

    // process "requested" roll/pitch into required lean angles.
    get_pilot_desired_lean_angles(target_roll, target_pitch, g2.ht_launch.get_dash_ang(), attitude_control->get_althold_lean_angle_max_cd());
}

void ModeLaunch::calc_althold_lean_req_nwse()
{
    // rotate pitch/roll by current yaw to NS/EW leans
    float NS_lean =  get_LP_pitch()*ahrs.cos_yaw() + get_LP_roll()*ahrs.sin_yaw();
    float EW_lean  = -get_LP_pitch()*ahrs.sin_yaw() + get_LP_roll()*ahrs.cos_yaw();
       // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = (float)ROLL_PITCH_YAW_INPUT_MAX / g2.ht_launch.get_dash_ang();
    NS_lean *= scaler;
    EW_lean *= scaler;

    // gcs().send_text(MAV_SEVERITY_WARNING, "px%2.1f rx%2.1f", _dash_NSlean, _dash_EWlean);
    // gcs().send_text(MAV_SEVERITY_WARNING, "TP1%2.1f TR1%2.1f", NS_lean, EW_lean);

    // these calcs look weird, because the x2-x1 section is by defn 1. because I am going to plot this against cos(t) instead of t
    _m_EWlean = (_dash_EWlean - EW_lean);
    _m_NSlean = (_dash_NSlean - NS_lean);
    _b_EWlean = _dash_EWlean - _m_EWlean;
    _b_NSlean = _dash_NSlean - _m_NSlean;

}
//
// IS_RELEASE_ALLOWED
// determines if release is allowed by checking the various params for release
// as of 250319, it checks yawRate, roll and pitch. more to come im sure.
//
ModeLaunch::eReleaseReason ModeLaunch::check_release_conditions(void)
{
            // Checks of No-Release Zone.
        // get vector from copter to home (or puck)
        Vector2f distvect ;
        if(!get_horiz_copter_to_target_vector( distvect )) return eReleaseReason::SafetyFence;

        float dist = distvect.length() ;

        // create unit vector.
        distvect.normalize() ;

        // check we are not pointing at home (or puck) position.
        float cur_heading = get_LP_yaw();
        // create heading vector.
        Vector2f headV( cosf(cur_heading), sinf(cur_heading) );

        //computer dot product of headvector and vector to copter.
        float dotp = headV * -distvect;
        // calculate the keep out (1/2) angle
        float keepoutZoneTheta = atan2f( g2.ht_launch.get_safety_fence_radius_m(), dist);
        // calculate the limit for the dotp.
        float keepoutLimit = -cosf(keepoutZoneTheta) ;

        // Verbose reporting.
        gcs().send_verbose_text(MAV_SEVERITY_CRITICAL,"dvx=%2.2f, dvy=%2.2f", -distvect.x, -distvect.y);
        gcs().send_verbose_text(MAV_SEVERITY_CRITICAL,"head=%2.2f", degrees(cur_heading));
        gcs().send_verbose_text(MAV_SEVERITY_CRITICAL,"hvx=%2.2f, hvy=%2.2f", headV.x, headV.y);
        gcs().send_verbose_text(MAV_SEVERITY_CRITICAL,"dotp=%2.2f", dotp);
        gcs().send_verbose_text(MAV_SEVERITY_CRITICAL,"kOth=%2.2f kOl=%2.2f", degrees(keepoutZoneTheta), keepoutLimit);

        //Logging
        AP::logger().Write( "ALRC",
                            "TimeUS,dvx,dvy, head,hvx,hvy,dotp, kOth, kOl",
                            "smmdmm-d-",
                            "F00000000",
                            "Qffffffff",
                            AP_HAL::micros64(),
                            -distvect.x,-distvect.y,
                            degrees(cur_heading),
                            headV.x, headV.y,
                            dotp,
                            degrees(keepoutZoneTheta), keepoutLimit);

        if (dotp < keepoutLimit) return eReleaseReason::ReleaseDir;
        // check if copter inside safety fence.
        if(dist < g2.ht_launch.get_safety_fence_radius_m()) return eReleaseReason::SafetyFence;

        // check servo state.
        if(_lock_servo != nullptr){
            if(_lock_servo->get_servo_safe())       return eReleaseReason::LockServoLocked;
        } else { return eReleaseReason::LockServoLocked; }
        // check change in yaw is not too high.
        if(fabsf(get_LP_yawrate()) > YAW_RATE_LIM)  return eReleaseReason::YawRateLP_Limited ;


        // check that lean angle is close enough to dash angle.
        float LeanLP = sqrtf( powf(get_LP_pitch(),2) + powf(get_LP_roll(),2) ) ;
        if(fabsf(LeanLP - g2.ht_launch.get_dash_ang()) > LEAN_ERROR_LIM)
                                                    return eReleaseReason::LeanLP_Limited;


        return eReleaseReason::ReleaseAllowed ;
}

bool ModeLaunch::allow_release(void)
{
    static uint32_t alerttime = 0 ;
    static const char* reasons[6] = { "allowed", "YawRateLim", "LeanLPLim", "Locked","SafetyFence","RelDir"} ;

    eReleaseReason allowed =  check_release_conditions();
    switch(allowed){
        case eReleaseReason::ReleaseAllowed: return true;
        default:
            if(( millis() - alerttime )>250 ) { // dont spam more often than every 250ms.
                gcs().send_text(MAV_SEVERITY_WARNING, "NO RELEASE(%s)", reasons[(int8_t) allowed] );
                alerttime = millis();
            }
            return false;
    }
}

//
// INIT_SERVOS
// initialize them to the correct settings for lock and un-release.
// set both servos to safe.
//
bool ModeLaunch::init_servos( void )
{

    _lock_servo = new SRV_Hood_HAL( SRV_Hood_HAL::k_lock ) ;
    if( _lock_servo == nullptr) { return false; }
    if( !_lock_servo->servo_found()) { return false; }

    _release_servo = new SRV_Hood_HAL(SRV_Hood_HAL::k_release ) ;
    if( _release_servo == nullptr ) { return false; }
    if( !_release_servo->servo_found()) { return false; }

    _release_alert = false;

    //reset the servo to locked.
    bool locked = _lock_servo->set_servo_safe() ;
    // close the _release_servo.
    bool closed = _release_servo->set_servo_safe();

    // if both servos were properly found, configured, and set, return true.
    return (closed && locked) ;

}



 // UPDATE_USER_OF_IMPENDING_RELEASE
// determine whether user should be alert to impending release.
// if they should be, do so.
void ModeLaunch::update_remain_release_window( uint32_t time )
{
    if(time<((uint32_t) g2.ht_launch.get_dash_time_ms())){
        // calculate time remaining till release.
        time = g2.ht_launch.get_dash_time_ms() - time ;

        // if not using auto-release
        if(!g2.ht_launch.autorel()){
            if( _lock_servo != nullptr ){
                if( !_lock_servo->get_servo_safe()){ // if lock servo is unlocked
                    // update the remaining time.
                    g2.ht_launch.set_remaining_rel_window(time) ;
                }
            }
        } else {
            // if using auto-release, just update the time to release.
            g2.ht_launch.set_remaining_rel_window(time) ;
        }
    } else {
        // if time is past for dashing, set remaining time to 0.
        g2.ht_launch.set_remaining_rel_window(0) ;
    }
}


//
// INIT_DASH_DIRECTION
// uses copters direction at arming to calculate the roll/pitch requirements
// for dashing in the direction of the copter at arming (plus any offset)
void ModeLaunch::init_dash_direction( int16_t angle_offset_deg )
{
    if (angle_offset_deg == 0 ){
        //fake like user is pressing on joystick, full right stick down.
        // essentially _dash_EWlean represents request to lean east(pos)/west(neg)
        // and pitch represents req to lean north(neg)/south(pos)
        _dash_EWlean =  0*copter.simple_cos_yaw - (STICK_FULL_FWD)*copter.simple_sin_yaw;
        _dash_NSlean = 0*copter.simple_sin_yaw + (STICK_FULL_FWD)*copter.simple_cos_yaw;
    }
    else
    {
        // add offset angle to the angle of arming via trig identities:
        // sin(a+b) = sin(a)cos(b) + cos(a)sin(b)
        // cos(a+b) = cos(a)cos(b) - sin(a)sin(b)
         _dash_EWlean = 0-(STICK_FULL_FWD)*(copter.simple_sin_yaw * cosf(radians(angle_offset_deg)) +
                                     copter.simple_cos_yaw * sinf(radians(angle_offset_deg))) ;

        _dash_NSlean = 0+(STICK_FULL_FWD)*(copter.simple_cos_yaw * cosf(radians(angle_offset_deg)) -
                                     copter.simple_sin_yaw * sinf(radians(angle_offset_deg))) ;

        // this code Im going to leave here in case we ever want to dash in an abs direction instead of offset from the current angle.
        // fake like user is pressing joystick such that it flies in direction of parameter angle.
        // _dash_EWlean = 0 - (STICK_FULL_FWD)*sinf(radians(g2.ht_launch.dash_direction));
        // _dash_NSlean = 0 + (STICK_FULL_FWD)*cosf(radians(g2.ht_launch.dash_direction));
    }

        gcs().send_text(MAV_SEVERITY_INFO, "NS%2.2f EW%2.2f",_dash_NSlean, _dash_EWlean );


}

// RUN_ALAUNCH_ONGROUND
// param target_roll - calculated roll target
// param target_pitch- calculated pitch target
// param target_climb_rate - input target climb rate to determine if user is attempting to set dash_direction.
//
void ModeLaunch::run_OnGround( float &target_roll, float &target_pitch, float target_climb_rate)
{
    attitude_control->reset_yaw_target_and_rate(false);
    attitude_control->reset_rate_controller_I_terms();

    // update target roll/pitch to do Takeoff above home position.
    target_roll =0 ;
    target_pitch = 0 ;

    loiter_nav->init_target();
    loiter_nav->set_pilot_desired_acceleration(0, 0);
    loiter_nav->clear_pilot_desired_acceleration();

    pos_control->set_externally_limited_xy();
    pos_control->relax_z_controller(0.0f);   // forces throttle output to go to zero

    loiter_nav->init_target( );


    // zero any pilot nudge.
    if( is_maritime())  g2.follow.zero_pilot_offsets();

    if (!motors->armed()) {
        // read if user has given input on which dir they want to go.
        read_dash_direction_from_remote(target_climb_rate);
    }
}



// RUN_ALAUNCH_TAKEOFF_FOLLOW
// param DES_VEL_NEU_CMS - calc desired vel
// param TARG_VEL - vel of target puck
// param DIST_VEC - dist vector btn puck and copter.
//
void ModeLaunch::run_Takeoff( Vector3f &des_vel_neu_cms, Vector3f &targ_vel, Vector3f &dist_vec )
{
    float target_climb_rate;
    // if takeoff isnt running yet, check that we are ready to go, then start takeoff.
    if( !takeoff.running()) {
        // if no direction was assigned via remote, then dont know where to go, disarm.
        if( _dash_dir == MaritimeDashDir::NotAssigned) {
            if( !AP_Notify::flags.flying //the copter thinks we are not "flying"
                && copter.ap.land_complete ) { //copter thinks we are on the ground.)
                    copter.arming.disarm(AP_Arming::Method::AUTOLAUNCH_TO_ERROR);
                gcs().send_text(MAV_SEVERITY_WARNING, "NO DIR ASSIGNED: DISARMING");
                return;
            }
        }

        // calculate and set the NED offsets for init asc.
        if(!set_autolaunch_NED_offsets())
        {
            if( !AP_Notify::flags.flying //the copter thinks we are not "flying"
                && copter.ap.land_complete ) { //copter thinks we are on the ground.)
                    copter.arming.disarm(AP_Arming::Method::AUTOLAUNCH_TO_ERROR);
                gcs().send_text(MAV_SEVERITY_WARNING, "NO OFFSETS: DISARMING");
                return;
            }
        }
        // POSSIBLE PROBLEM HERE: what if the boat turns?
        // init dash angles based on takeoff heading.
        init_dash_direction( get_maritime_dash_dir_angle() );

        // start takeoff by calling for motors up.
        takeoff.start(constrain_float(g.pilot_takeoff_alt,TAKEOFF_MIN_ALT,TAKEOFF_MAX_ALT));
    }
    // constrain target climb rate with user specified speed.
    target_climb_rate = constrain_float( g.pilot_speed_up, 0, g.pilot_speed_up);

    // get take-off adjusted pilot and takeoff climb rates
    takeoff.do_pilot_takeoff(target_climb_rate); 

    // match target velocities to the target, and target a gentle ascent rate.
    des_vel_neu_cms.x = (targ_vel.x * 100.0f);
    des_vel_neu_cms.y = (targ_vel.y * 100.0f);
    des_vel_neu_cms.z = (target_climb_rate);  //positive because guided mode vel controller is NEU not NED...like the rest of follow? wtf?

    // just in case it gets in the air, but doesnt go through the normal progression of grd->TO-?flying
    if(dist_vec.z > 2) {
        gcs().send_text(MAV_SEVERITY_WARNING, "force InitAsc");
        init_dash_direction( get_maritime_dash_dir_angle() );
        if(set_autolaunch_NED_offsets()) _state_complete = true;
        else { send_copter_home(); }
    }
}


void ModeLaunch::run_InitAsc(Vector3f &des_vel_neu_cms, Vector3f &targ_vel, Vector3f &dist_vec, Vector3f &req_pilot_vel)
{
    float horiz_vel_lim = _InitAsc_exit_vel;

    dist_vec += _ALoffsetNED ;

    //FollowModeAccess.calc_des_vel( des_vel_neu_cms, dist_vec, targ_vel, req_pilot_vel ) ;
    FollowModeAccess.calc_des_vel( des_vel_neu_cms, dist_vec, targ_vel, req_pilot_vel ) ;

    if( _state_time < VERT_ASC_MS) {
        // for first 2sec of takeoff, go straight up - no horiz vel.
        horiz_vel_lim = 0 ;

    } else if(( _state_time-VERT_ASC_MS)<INC_HORZ_VEL_TIME_MS) {
        horiz_vel_lim = cosine_increase_decrease( (_state_time-VERT_ASC_MS), 2*INC_HORZ_VEL_TIME_MS, 1, 0) ;
        horiz_vel_lim = constrain_float(horiz_vel_lim,0,1);
        horiz_vel_lim *= _InitAsc_exit_vel;
    }

    Vector2f des_horiz_vel(des_vel_neu_cms.x, des_vel_neu_cms.y);
    des_horiz_vel.normalize();

    des_vel_neu_cms.x = horiz_vel_lim*des_horiz_vel.x ;
    des_vel_neu_cms.y = horiz_vel_lim*des_horiz_vel.y ;

    if( des_vel_neu_cms.z > 0 ) {
        des_vel_neu_cms.z = g.pilot_speed_up ;
    }

    // check if copter has reached slow down dist.
    if ((fabsf(dist_vec.z) < _slow_down_z_dist)) _state_complete=true;
 

    // DEBUG.  this whole thing should be removed for release.
    if(true){
        AP::logger().Write( "ALIA",
                            "TimeUS,ALOx,ALOy,ALOz,Dx,Dy,Dz,DVx,DVy,DVz,Tx,Ty",
                            "smmmmmmnnnnn",
                            "F------BBB--",
                            "Qfffffffffff",
                            AP_HAL::micros64(),
                            _ALoffsetNED.x,
                            _ALoffsetNED.y,
                            _ALoffsetNED.z,
                            dist_vec.x,
                            dist_vec.y,
                            dist_vec.z,
                            des_vel_neu_cms.x,
                            des_vel_neu_cms.y,
                            des_vel_neu_cms.z,
                            targ_vel.x,
                            targ_vel.y);
    }


}

void ModeLaunch::run_SlowToDashAlt(Vector3f &des_vel_neu_cms, Vector3f &targ_vel, Vector3f &dist_vec, Vector3f &req_pilot_vel)
{
    float horiz_vel_lim = _InitAsc_exit_vel;

    dist_vec += _ALoffsetNED ;

    //FollowModeAccess.calc_des_vel( des_vel_neu_cms, dist_vec, targ_vel, req_pilot_vel ) ;
    FollowModeAccess.calc_des_vel( des_vel_neu_cms, dist_vec, targ_vel, req_pilot_vel ) ;

    Vector2f des_horiz_vel(des_vel_neu_cms.x, des_vel_neu_cms.y);
    des_horiz_vel.normalize();

    des_vel_neu_cms.x = horiz_vel_lim*des_horiz_vel.x ;
    des_vel_neu_cms.y = horiz_vel_lim*des_horiz_vel.y ;

    // calc desired (slowing down) Vel-z
    des_vel_neu_cms.z =  cosine_increase_decrease( _state_time, 2*g2.ht_launch.get_predash_ms(),  -g.pilot_speed_up, g.pilot_speed_up ) ;

    if(_state_time > g2.ht_launch.get_predash_ms()) _state_complete=true;
}


// RUN_ALAUNCH_PREDASH
// computes target_roll and target_pitch for Pre-Dash or Half-Dash
void ModeLaunch::run_PreDash( float &target_roll, float &target_pitch, Vector3f &des_vel )
{
    // (re)set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float cost = cosine_increase_decrease(_state_time,2*g2.ht_launch.get_predash_ms(),1,0);
    float NS_lean = _m_NSlean * cost + _b_NSlean;
    float EW_lean = _m_EWlean * cost + _b_EWlean;

    // simple mode translation of NS/EW lean
    channel_roll->set_control_in(  EW_lean*ahrs.cos_yaw() + NS_lean*ahrs.sin_yaw());
    channel_pitch->set_control_in(-EW_lean*ahrs.sin_yaw() + NS_lean*ahrs.cos_yaw());

    // process "requested" roll/pitch into required lean angles.
    get_pilot_desired_lean_angles(target_roll, target_pitch, g2.ht_launch.get_dash_ang(), attitude_control->get_althold_lean_angle_max_cd());

    // check if predash is done.
    if( _state_time >= g2.ht_launch.get_predash_ms()) {
        _state_complete = true ;
    }
        AP::logger().Write( "ALPD",
                            "TimeUS,cost,NSlean,EWlean,cR,cP,tR,tP",
                            "ssdddddd",
                            "F0BBBBBB",
                            "Qfffffff",
                            AP_HAL::micros64(),
                            cost,
                            NS_lean,
                            EW_lean,
                            EW_lean*ahrs.cos_yaw() + NS_lean*ahrs.sin_yaw(),
                            -EW_lean*ahrs.sin_yaw() + NS_lean*ahrs.cos_yaw(),
                            target_roll,
                            target_pitch
                     );
}


// RUN_ALAUNCH_DASH
// computes target_roll and target_pitch for Dash
void ModeLaunch::run_Dash( float &target_roll, float &target_pitch )
{
    // (re)set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // update the user on the HUD of impending release.
    update_remain_release_window( _state_time);

    // check if done with dash.
    if(_state_time > g2.ht_launch.get_dash_time_ms()) _state_complete=true;

    // servo checks.
    if( _lock_servo != nullptr ){
        // if its unlock time and the servo is in safe position, go unsafe.
        if(( _state_time > MIN_DASH_BEF_UNLOCK_MS)
                && (_lock_servo->get_servo_safe())
                && (_state_time < g2.ht_launch.get_dash_time_ms())){
            // UNLOCK.
            if(_lock_servo->set_servo_unsafe()){
                if( !_lock_servo->get_servo_safe() ) {
                    // set state so telem lets MP know we are unsafe.
                    g2.ht_launch.set_lockservo_state(COMMUNIC_UNSAFE) ;
                    gcs().send_text(MAV_SEVERITY_WARNING,"UNLOCKED") ;
                }
            }
        }
    }

    // if in manual release mode
    if (!g2.ht_launch.autorel()) {
        // if user has hit release and the servo has gone unsafe, note that we released.
        if(_release_servo != nullptr ) {
            if (!_release_servo->get_servo_safe()) _released = true ;

            //user hit release and it opened, and now user has let go of release button.
            if (_released && (_release_servo->get_servo_safe()))  _state_complete=true;
        }
    }

    // calc target roll/pitch for this point in dash.
    calc_angles_for_dash( 1.0f, _ascent_rate, target_roll, target_pitch ) ;

}

void ModeLaunch::run_Release( float &target_roll, float &target_pitch )
{
    if( _state_time > AFTER_RELEASE_PULLUP_MS ) {
        _state_complete = true;
        if(_lock_servo != nullptr ) {
            if (!_lock_servo->get_servo_safe()) {
                if(_lock_servo->set_servo_safe()) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "LOCKED") ;
                }
            }
        }
    }
    if( g2.ht_launch.autorel()) {
        //RELEASE.
        attempt_to_release();
    }

    // calc target roll/pitch for this point in (pre)Dash
    calc_angles_for_dash( 1.0f, 0.0f, target_roll, target_pitch ) ;

}


// RUN_POSTDASH
// show slow down and ascend copter slightly.
// calcs roll/pitch to accomplish this.
void ModeLaunch::run_PostDash( float &target_roll, float &target_pitch )
{

    //TODO: _dash_factor could become local variable in PostDash but right now Im logging it, and until we feel
    // comfortable with everything I'll leave it for logging porpoises. -losh, 240521

    if (_state_time > AFTER_DASH_RTL_TIME) {
        _state_complete = true;
    } else {
    //SLOW DOWN
        _dash_factor = cosine_increase_decrease(    _state_time,
                                                    2*AFTER_DASH_RTL_TIME ,
                                                    -1,    // multiply by -1 to flip it,
                                                    1 ) ;  // and add 1, so you start at 1, and come down to 0.
    }
     // calc and constrain the dash_factor.  make sure not crazy values get through.
    _dash_factor =  constrain_float( _dash_factor, 0.0f, 1.0f) ;

    // calc target roll/pitch for this point in dash.
    calc_angles_for_dash( _dash_factor, _ascent_rate, target_roll, target_pitch ) ;
}

void ModeLaunch::start_InitAsc(void)
{
    // attempt to turn on blower if reqd (configured).
    g2.ht_launch.blower_on();

    // calculate vertical slow down dist for commanded cosine decay of vel from max->0 over predash time.
    _slow_down_z_dist = (fabsf(g.pilot_speed_up)/100) * (2*(float)(g2.ht_launch.get_predash_ms())/1000) / 4 ;

    // calc Init ascent horiz "exit" vel that matches speed with about how long it will take to reach dash alt.
     _InitAsc_exit_vel = (float)g2.ht_launch.get_offset() / ((float)g2.ht_launch.get_dash_alt_m() / (float)g.pilot_speed_up); //spd in cm/s

    //DEBUG
    gcs().send_verbose_text(MAV_SEVERITY_WARNING,"Vexit=%2.2f",  _InitAsc_exit_vel) ;
}

void ModeLaunch::start_SlowToDashAlt(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING,"SlowVz");

    // dont keep ascending.
    pos_control->init_z_controller();
}
void ModeLaunch::start_PreDash(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING,"PRE-DASH");

    calc_althold_lean_req_nwse();

    // init pos controller.
    pos_control->init_z_controller();
}


void ModeLaunch::start_Dash(void)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    gcs().send_text(MAV_SEVERITY_WARNING,"DASHING");
}


void ModeLaunch::start_PostDash(void)
{
    //note time of post dash, so blower can be turned off.
    g2.ht_launch.set_blower_off_time();

    gcs().send_text(MAV_SEVERITY_WARNING,"POST-DASHING");

    if(_release_servo != nullptr ) {
        // close the release servo.
        _release_servo->set_servo_safe() ;
    }
    // set release window to 0.
    g2.ht_launch.set_remaining_rel_window(0) ;
}

// bool ModeLaunch::is_maritime(void)
// {
//     return (g2.follow.enabled() && ( g.flight_mode5.get() == (signed char) Mode::Number::FOLLOW )) ;
// }

//
// GET_AUTOLAUNCH_NED_OFFSETS
// compute offsets in NED from offset_angle and offset params that are in copter coords.
// returns true if offset was sucessfully calculated.
// ALoffset param is the path to return the offsets in NED
bool ModeLaunch::set_autolaunch_NED_offsets( void )
{
    Vector3f off;
    float angle_deg=0.0f;

    if (is_maritime()) {
        // returns 360 if not assigned yet.
        angle_deg = (float) get_maritime_offset_angle();
        if(angle_deg>359.0f) {
            _ALoffsetNED.zero();
            gcs().send_text(MAV_SEVERITY_WARNING, "angle>359");
            return false;
        }
    } else {
        //angle_deg = (float) g2.ht_launch.AL_dash_direction_offset ;
        angle_deg=0;
    }

    // calculate offset distances in copter coords
    // increase horizontal offsets to account for slowing down in follow mode never really making it to the location.
    off.x = HORZ_OFFSET_INC * g2.ht_launch.get_offset() * cosf(radians(angle_deg)) ;
    off.y = HORZ_OFFSET_INC * g2.ht_launch.get_offset() * sinf(radians(angle_deg)) ;

    //Debug output.
    gcs().send_verbose_text(MAV_SEVERITY_ALERT, "A:%1.1f x:%1.1f y:%1.1f", angle_deg, off.x, off.y)  ;

    // because we are working NED, up is negative so I negate the dash_alt.
    // extra 1% is just to make sure we are dash alt not below it.
    off.z = (-1.01)*g2.ht_launch.get_dash_alt_m() ; 

    // if offsets are zero, simply return offset vector
    if (off.is_zero()) {
        _ALoffsetNED = off;
        return true;
    }

    // rotate the offset vector from copter frame to NED
    _ALoffsetNED = Vector3f( off.x*ahrs.cos_yaw() - off.y*ahrs.sin_yaw(),
                            off.y*ahrs.cos_yaw() + off.x*ahrs.sin_yaw(),
                            off.z);
    //debug output.
    gcs().send_verbose_text(MAV_SEVERITY_ALERT, "cY:%1.2f sY:%1.2f", ahrs.cos_yaw(), ahrs.sin_yaw())  ;
    gcs().send_verbose_text(MAV_SEVERITY_ALERT, "N:%1.1f E:%1.1f", _ALoffsetNED.x, _ALoffsetNED.y)  ;

    return true;

}

// GET_DASH_DIRECTION_FROM_REMOTE
/// @brief
/// @param
void ModeLaunch::read_dash_direction_from_remote( float target_climb_rate )
{
    int16_t roll;
    static int8_t right = 0 ;
    static int8_t left  = 0 ;
    static int8_t fwd   = 0 ;

    //direction strings.  this should jive with order of MaritimeDarDir enum.
    static const char* dirs[4] = { "notassigned", "LEFT", "FORWARD", "RIGHT"} ;

    // read joystick input on right stick.
    roll =  channel_roll->get_control_in();

    // check that throttle is held down.
    if( target_climb_rate < 0.98*-get_pilot_speed_dn()) {

        // check if pilot holding stick right.
        if((roll>MINIMUM_RC_VAL) && is_maritime()) {
            //increment debounce counter.
            right++;
            // if enough instances, call that a dir.
            if (right>DEBOUNCE_N){
                _dash_dir = MaritimeDashDir::CopterRight;
                right=0;
            }
        // check if held stick left.
        } else if ((roll< (-MINIMUM_RC_VAL)) && is_maritime() ) {
            //increment debounce counter.
            left++;
            // if enough instances, call that a dir.
            if (left>DEBOUNCE_N){
                _dash_dir = MaritimeDashDir::CopterLeft;
                left=0;
            }
        } else if (( channel_pitch->get_control_in() < (-MINIMUM_RC_VAL)) && !is_maritime() ) {
            //increment debounce counter.
            fwd++;
            // if enough instances, call that a dir.
            if (fwd>DEBOUNCE_N){
                _dash_dir = MaritimeDashDir::CopterFwd;
                fwd=0;
            }

        } else {
            //if no large enough stick action, reset debounce counters.
            right=0; left=0; fwd=0;
        }
    } else { //pilot not holding down the throttle, reset things.
        right=0; left=0; fwd=0;
        _dash_dir = MaritimeDashDir::NotAssigned ;
    }

    // if dash dir changed, tell pilot at GCS.
    if (_dash_dir != _last_dash_dir){
        _last_dash_dir = _dash_dir;

        // set for relaying via MavLink.
        g2.ht_launch.set_dash_dir((int8_t) _dash_dir) ;

        if( _dash_dir != MaritimeDashDir::NotAssigned){
            if(!set_autolaunch_NED_offsets()){
                gcs().send_text(MAV_SEVERITY_WARNING, "COULD NOT SET OFFSETS") ;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "DEPARTING COPTER %s", dirs[_dash_dir]) ;
            }
        } else { gcs().send_text(MAV_SEVERITY_WARNING, "Dash Dir Reset") ;}
    }
}


// RETURN_ANGLE_FROM_dash_dir
int16_t ModeLaunch::get_maritime_dash_dir_angle( void )
{
    int16_t angle = get_maritime_offset_angle() ;

    return angle;

    // CODE to add some angle dependent on boat velocity.
/*
    if (angle==0) return angle;


    Vector3f velned;
    if(g2.follow.get_velocity_ned( velned, G_Dt )) {
        float velocity;
        int16_t add_angle;
        Vector2f vel_ne( velned.x, velned.y) ;
        velocity = vel_ne.length() ;
        gcs().send_verbose_text(MAV_SEVERITY_ALERT, "vel:%2.2f", velocity) ;

        if(velocity < 0.5) add_angle = 0;
        else if( velocity<10)    add_angle = (int16_t) (6.0f * velocity);
        else add_angle = 45;

        if( angle < 0) add_angle = -add_angle;
        gcs().send_verbose_text(MAV_SEVERITY_ALERT, "dash< %d", add_angle+angle);
        return (angle+add_angle);
    }
    return angle;
    */
}

int16_t ModeLaunch::get_maritime_offset_angle(void)
{
    switch(_dash_dir) {
    case NotAssigned:
        return 360;
        break;
    case CopterLeft:
        return (-90) ;
        break;
    case CopterRight:
        return 90 ;
        break;
    default:
        return 0 ;
    }
}


bool ModeLaunch::check_parameters(void)
{
        hal.console->printf("AL param check\n");

    if(!g2.ht_launch.check_parameters())
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "Check Launch Params") ;
        return false;
    }
    return true;
}

#endif