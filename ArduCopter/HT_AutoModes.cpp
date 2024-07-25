/*
Common Hood AUTO MODE Stuff
methods common to all hood auto modes.

Hood Technology Jan 2024
-losh
*/
#include "Copter.h"

#if(( MODE_LAUNCH_ENABLED==ENABLED)||(MODE_RECOVERY_ENABLED==ENABLED))
#include <AP_GPS/AP_GPS.h>
#include "HT_AutoModes.h"

bool ModeHoodAuto::init(bool ignore_checks)
{
    hal.console->printf("HoodAuto init\n");
    // if already flying.
    if( !copter.ap.land_complete) return false;

    if(copter.arming.is_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "ALREADY ARMED") ;
        return false;
    }

    if(!check_parameters()) return false;

    if (!copter.current_loc.initialised()) {
        gcs().send_text(MAV_SEVERITY_WARNING,"AR: loc not init.");
        return false ;
    }


    // if dont have a home.
    if (!AP::ahrs().home_is_set()) return false;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    // get dist_vec and vel_of_target in meters and NED.
    if (!get_target_dist_and_velocity_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        if(is_maritime()){
            if(dist_vec.z>=0){
                gcs().send_text(MAV_SEVERITY_WARNING,"MHA: FOLL_OFS_Z>=0" ) ;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING,"No puck" ) ;
            }
        }
        else gcs().send_text(MAV_SEVERITY_WARNING,"No home loc yet" ) ;
        // return false;
    }
        g2.follow.zero_pilot_offsets();
        
    if(is_maritime())
    {


        // HOODTECH MOD, losh 220907, check offsets.
        // get offsets
        Vector3f init_offsets;
        if (!g2.follow.get_offsets_ned(init_offsets)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "FOLL_init: No Offsets.");
            return false;
        }
        if( init_offsets.z>=0){
            gcs().send_text(MAV_SEVERITY_WARNING,"non-negative Zoffset");
            return false;
        }
        if(!ModeGuided::init(ignore_checks)) return false;
    }

    // init the WPnav module.
    wp_nav->wp_and_spline_init();


    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // if pos control hasnt been used recently, init it.
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller_no_descent();
    }
    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);
    stopping_point.x = stopping_point.x +100.0f;
    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    low_pass_ahrs(true) ;
    return true;
}

bool ModeHoodAuto::ready_to_arm()
{
    if(!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Follow Mode Disabled");
        return false;
    }

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    // get dist_vec and vel_of_target in meters and NED.
    if (!get_target_dist_and_velocity_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        gcs().send_text(MAV_SEVERITY_ERROR,"No Target Link" ) ;
        //follow link is not set up yet
        return false;
    }
    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)){
        //ahrs is in a bad state somehow
        return false;
    }

    //if land based setup, make sure vehicle isnt moving because RTL will be bad if really on ship.
    if(!is_maritime()){
        float Vxy = safe_sqrt( powf(veh_velocity.x,2)+ powf(veh_velocity.y,2)) ;
        if ( Vxy > g2.follow.max_arming_vel_mismatch() ){
            gcs().send_text(MAV_SEVERITY_ERROR,"V%2.2f != 0, Maritime?", Vxy ) ;
            return false;
        }
    } else {
        Vector3f vel_dif = veh_velocity-vel_of_target;
        if (vel_dif.length()>g2.follow.max_arming_vel_mismatch()){
            gcs().send_text(MAV_SEVERITY_ERROR," VelMisMatch" ) ;
            //difference of velocity is too great to safely take off
            return false;
        }
        // if arming in Follow reset pilot offsets. This is for edge case of landing in follow mode, then not changing modes at all
        // and then taking off again follow mode.  In this case, if we dont reset offsets, the copter cant take off cause pilot
        // offsets are keeping it on the grd.
        g2.follow.zero_pilot_offsets();
    }

    if (veh_velocity.length()>g2.follow.max_arming_vel()){
        gcs().send_text(MAV_SEVERITY_ERROR," Vel > VelMax" ) ;
        //veh_velocity is too high to safely take off
        return false;
    }

    //if all checks are ok, then ready to arm.
    return true;
}
bool ModeHoodAuto::is_maritime(void)
{
    return (g2.follow.enabled() && ( g.flight_mode5.get() == (signed char) Mode::Number::FOLLOW )) ;
}

bool ModeHoodAuto::get_target_dist_and_velocity_ned( Vector3f &dist_v, Vector3f &dist_v_offset, Vector3f &vel_of_targ )
{
    if(is_maritime()) {
        return g2.follow.get_target_dist_and_vel_ned(dist_v, dist_v_offset, vel_of_targ) ;
    } else {
        //fake out dist vector using home position as "puck" loc for land based ops.
        if(!AP::ahrs().get_relative_position_NED_home(dist_v)) return false;
        dist_v *= (-1) ; //scale by -1 to flip vector direction.

        dist_v_offset = dist_v;
        vel_of_targ.zero();
    }
    return true;
}

bool ModeHoodAuto::get_diff_vec( Vector3f &diffvec )
{
    if( is_maritime() ) {
        // get our location
        Location current_loc;
        if (!AP::ahrs().get_location(current_loc)) return false;

        // get target location and velocity
        Location target_loc;
        Vector3f veh_vel;
        if (!g2.follow.get_target_location_and_velocity(target_loc, veh_vel)) return false;

        // change to altitude above home if relative altitude is being used
        if (target_loc.relative_alt == 1) {
            current_loc.alt -= AP::ahrs().get_home().alt;
        }
        // calculate difference between current loc and target (gps puck) location
        diffvec = current_loc.get_distance_NED(target_loc);

    } else {
        // calc vector from home to current position.
        if(!AP::ahrs().get_relative_position_NED_home(diffvec)) return false;
        diffvec *= (-1) ; //scale by -1 to flip vector direction.
    }
    return true;
}


//
// COSINE_INCREASE_DECREASE.
//varies output by cosine that goes from 0->1->0 with period T.
// scales amp by parameter AMP, and offsets the entire output by parameter OFFSET
//
float ModeHoodAuto::cosine_increase_decrease( float t, float T, float amp, float offset )
{
    return ( amp*(0.5 - 0.5*cosf( 6.28*( t/T ) )) + offset) ;

}

//
// COSINE_CHANGE.
//varies output by cosine
// scales from start to end via cosine over countdown.
//
float ModeHoodAuto::cosine_change( int32_t t_countdown, int32_t T_ticks,  float start, float end )
{
     return ( (end-start)*(0.5 - 0.5*cosf( 6.28*( (T_ticks - t_countdown)/(2.0f*T_ticks) ) )) + start) ;
}


bool ModeHoodAuto::get_horiz_copter_to_home_vector( Vector2f &vec)
{
       //should do same thing here but from copter to home position.
        Location home_location = ahrs.get_home();

        Location current_loc;
        if (!AP::ahrs().get_location(current_loc)) {
            return false;
        }
        // calculate difference
        Vector3f copter_to_home_vector_NED = current_loc.get_distance_NED(home_location);

        vec = copter_to_home_vector_NED.xy();

        return true;
}

// GET_HORIZ_COPTER_TO_TARGET_VECTOR
// calcs vector from copter to base (puck or home)
//
bool ModeHoodAuto::get_horiz_copter_to_target_vector(Vector2f &copter_to_target_vector)
{
    if (is_maritime()){
        // retrieve 2d vector from copter to puck
        if(!g2.follow.get_horz_target_dist_ned(copter_to_target_vector) ){
            return false;
        }
    } else {
        if(!get_horiz_copter_to_home_vector(copter_to_target_vector)) {
            return false;
        }
    }
    return true;
}

//
// SEND_COPTER_HOME
// chooses correct mode (RTL or FOLLOW) based on configuration at end of mission.
//
bool ModeHoodAuto::send_copter_home(void)
{
    // depending on configuration, send copter home or to the follow puck.
    if( is_maritime()){
        //set follow mode.
        // will need more work, to go home to follow puck and descend?
        return copter.set_mode(Mode::Number::FOLLOW, ModeReason::RC_COMMAND);
    }
    else {
        // set RTL mode
        return copter.set_mode(Mode::Number::RTL, ModeReason::RC_COMMAND);
    }
}




Vector3f ModeHoodAuto::update_pilot_nudge(Vector3f &req_pilot_vel)
{
        Vector3f pilot_vel;
        pilot_vel.zero();
        if (!copter.failsafe.radio) {

            // scale to circular angle max limit
            //get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
            float target_roll = channel_roll->get_control_in()*copter.simple_cos_yaw - channel_pitch->get_control_in()*copter.simple_sin_yaw;
            float target_pitch = channel_roll->get_control_in()*copter.simple_sin_yaw + channel_pitch->get_control_in()*copter.simple_cos_yaw;

            // pilot inputs to lat/lon frame   MEK
            if(fabsf(target_pitch)>100) {
                pilot_vel.x = (-target_pitch)*radians(0.01f*g2.follow.pilot_max_xy());
                req_pilot_vel.x = pilot_vel.x;
            }
            if(fabsf(target_roll)>100){
                pilot_vel.y = (target_roll)*radians(0.01f*g2.follow.pilot_max_xy());
                req_pilot_vel.y = pilot_vel.y;
            }

            // update the nudges in the follow world.
            g2.follow.update_pilot_nudge_xy( pilot_vel, G_Dt ) ;
            // read the updated nudges from the follow world.
            Vector3f offset_nudges = g2.follow.get_pilot_offset_nudges() ;
            // return the nudges back to the main run().
            return( offset_nudges) ;
        }
        return(pilot_vel) ; // return 0s.
}


void ModeHoodAuto::check_for_landing(void)
{
    // if landing detected, spool down in anticipation of disarming.
    if (copter.ap.land_complete )
    {
        make_safe_ground_handling();
    }

    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "DISARMING") ;
        copter.arming.disarm(AP_Arming::Method::LANDED);
         copter.set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND);

    }
}



uint32_t ModeHoodAuto::wp_distance() const
{
    return (uint32_t) (100*_dist_to_target);
}

int32_t ModeHoodAuto::wp_bearing() const
{
    return (int32_t) 100*_bearing_to_target;
}

/*
  get target position for mavlink reporting
 */
bool ModeHoodAuto::get_wp(Location &loc) const
{
    loc = copter.current_loc;
    loc.offset_bearing(_bearing_to_target, _dist_to_target);
    return true;

}

void ModeHoodAuto::low_pass_ahrs(float old_fact, float new_fact)
{

     // calculate the low-passed version of yaw_rate so we can check that the copter yaw has settled out.
    _yaw_rate_LP = _yaw_rate_LP*old_fact + new_fact * AP::ahrs().get_gyro().z ;
    // calc low-passed roll for roll checks before release.
    _roll_LP = _roll_LP * old_fact + new_fact * AP::ahrs().roll_sensor ;
    // calc low-passed pitch for pitch checks before release.
    _pitch_LP = _pitch_LP * old_fact + new_fact * AP::ahrs().pitch_sensor ;
    // low-passed yaw in RADIANS.
    _yaw_LP = _yaw_LP * old_fact + new_fact * AP::ahrs().get_yaw();

    static uint32_t now = millis();

    if((millis() - now)>100){
        AP::logger().Write( "AULP",     //log category name.
                            "TimeUS,yawRatLP,RollLP,PitLP,YawLP",    //names
                            "sEddr",   //units
                            "F0BB0",   //scaling
                            "Qffff",   //format
                            AP_HAL::micros64(),
                            _yaw_rate_LP,
                            _roll_LP,
                            _pitch_LP,
                            _yaw_LP );
    }
}

void ModeHoodAuto::low_pass_ahrs(bool reset)
{
     if(reset) {
        _yaw_rate_LP = 0 ;
        _roll_LP = 0 ;
        _pitch_LP = 0 ;
        _yaw_LP = 0 ;
    }
}
























#endif