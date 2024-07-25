#include "Copter.h"

#if MODE_FOLLOW_ENABLED == ENABLED


// HoodTech defns
//
# define ALLOW_UP_NUDGE_ALT_PERCENT 0.5 //determines where upward stick will result in changing accumulated pilot nudge.
# define VEL_NUDGE_DEADBAND 0.1f //determines deadband range of pilot stick that is considered 0 (no) input.
# define AP_FOLLOW_STICK_LP          150   // HOODTECH MOD, losh 210912, 0=LP off.

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AP_AVOIDANCE_ENABLED is true because we rely on it velocity limiting functions
 */

// initialise follow mode
bool ModeFollow::init(const bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP_Mount::get_singleton();
    // follow the lead vehicle using sysid
    if (g2.follow.option_is_enabled(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER) && mount != nullptr) {
        mount->set_target_sysid(g2.follow.get_target_sysid());
    }
#endif

    g2.follow.zero_pilot_offsets();

   if(!g2.follow.set_use_return_alt()) {
        gcs().send_text(MAV_SEVERITY_WARNING,"COULD NOT SET OFF-Z") ; 
        return false;
   }

    // HOODTECH MOD, losh 220907, retrieve offsets, so Zoffset can be used later.
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

    old_pilot_vel_z = 0;

    //HOODTECH MOD, losh 220831, init pilot_nudging to keep track of what the pilot has been doing.
    pilot_nudging = false ;
    allow_upward_nudging = false ;  //since copter is taking off, reset allow up nudge.
    Z_alt_up_nudge_allowed = abs(init_offsets.z) * ALLOW_UP_NUDGE_ALT_PERCENT ;
    gcs().send_verbose_text(MAV_SEVERITY_WARNING,"Zalt=%2.1f",Z_alt_up_nudge_allowed);

    // re-use guided mode
    return ModeGuided::init(ignore_checks);
}

//checks if follow mode is ready to arm
// 1) follow messages are arriving
// 2) velocity is within limit
// 3) follow velocity is consistent with vehicle velocity
bool ModeFollow::ready_to_arm()
{
    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (!g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "FOLL:no link.");
        //follow link is not set up yet
        return false;
    }
    Vector3f veh_velocity;
    if (!AP::ahrs().get_velocity_NED(veh_velocity)){
        gcs().send_text(MAV_SEVERITY_WARNING, "FOLL:bad ahrs.");
        return false;
    }

    if (veh_velocity.length()>g2.follow.max_arming_vel()){
        gcs().send_text(MAV_SEVERITY_WARNING, "FOLL:Veh vel too high.");
        //veh_velocity is too high to safely take off
        return false;
    }

    Vector3f vel_dif = veh_velocity-vel_of_target;
    if (vel_dif.length()>g2.follow.max_arming_vel_mismatch()){
        gcs().send_text(MAV_SEVERITY_WARNING, "FOLL:vel mismatch.");
        //difference of velocity is too great to safely take off
        return false;
    }

    // if arming in Follow reset pilot offsets. This is for edge case of landing in follow mode, then not changing modes at all
    // and then taking off again follow mode.  In this case, if we dont reset offsets, the copter cant take off cause pilot
    // offsets are keeping it on the grd.
    g2.follow.zero_pilot_offsets();

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFollow::run()
{

    float target_roll, target_pitch;
    Vector3f _pilot_vel;


    if (!copter.failsafe.radio) {

        // scale to circular angle max limit
        //get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
        target_roll = channel_roll->get_control_in()*copter.simple_cos_yaw - channel_pitch->get_control_in()*copter.simple_sin_yaw;
        target_pitch = channel_roll->get_control_in()*copter.simple_sin_yaw + channel_pitch->get_control_in()*copter.simple_cos_yaw;

        // pilot inputs to lat/lon frame   MEK
         _pilot_vel.x = (-target_pitch)*radians(0.01f*g2.follow.pilot_max_xy());
         _pilot_vel.y = (target_roll)*radians(0.01f*g2.follow.pilot_max_xy());

        _pilot_vel.z = -get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        _pilot_vel.z = constrain_float(_pilot_vel.z, -get_pilot_speed_dn(), g.pilot_speed_up);
        _pilot_vel.z /= 100.0f; //convert from cm/s

        // HOODTECH MOD losh 220912, adding Lowpass of pilot inputs.
        // best I can tell, run() runs at 400Hz.  that says stick_LP of 400 equals a LP of 1Hz.
        if (AP_FOLLOW_STICK_LP>0)
        {
            _pilot_vel.z = _pilot_vel.z * (1/(float)AP_FOLLOW_STICK_LP) + old_pilot_vel_z * ((AP_FOLLOW_STICK_LP-1)/(float)AP_FOLLOW_STICK_LP) ;
            old_pilot_vel_z = _pilot_vel.z;
        }


    }

    AltHoldModeState follow_state = get_alt_hold_state(-_pilot_vel.z*100.0f); //negative because we're working in positive up here

    static AltHoldModeState last_state = AltHoldModeState::Flying;
    if (follow_state!=last_state){
        last_state = follow_state;
    }

    //assume state won't be disarmed or landed at this point
    if ((follow_state==AltHoldModeState::Landed_Ground_Idle || follow_state == AltHoldModeState::Landed_Pre_Takeoff) && -_pilot_vel.z>0){
        follow_state = AltHoldModeState::Takeoff;

    }

    // #TODO: we took this out before, but should I figure out how to put it back?
    // // if not armed set throttle to zero and exit immediately
    // if (is_disarmed_or_landed()) {
    //     make_safe_ground_handling();
    //     return;
    // }


    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    // variables to be sent to velocity controller
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {

        switch (follow_state)
        {
        //top 3 cases replicate the previous behavior of exiting further up.
        case AltHoldModeState::Landed_Ground_Idle:
            allow_upward_nudging = false;
            FALLTHROUGH ;
        case AltHoldModeState::Landed_Pre_Takeoff:
            desired_velocity_neu_cms.x = (vel_of_target.x * 100.0f);
            desired_velocity_neu_cms.y = (vel_of_target.y * 100.0f);
            allow_upward_nudging = false;
            break;
        case AltHoldModeState::MotorStopped:
            make_safe_ground_handling();
            allow_upward_nudging = false;
            return;
            break;


        case AltHoldModeState::Takeoff:
            //vehicle velocity is checked to be close to target velocity at arming
            //if there is a significant speed mismatch between vehicle and follow target
            //tipover seems likely
            if (!takeoff.running()) takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));

            desired_velocity_neu_cms.x = (vel_of_target.x * 100.0f);
            desired_velocity_neu_cms.y = (vel_of_target.y * 100.0f);
            desired_velocity_neu_cms.z = (-_pilot_vel.z * 100.0f);

            takeoff.do_pilot_takeoff( _pilot_vel.z );

            // HOODTECH MOD -losh, 220905
            allow_upward_nudging = false ;  //since copter is taking off, reset allow up nudge.
            break;


        case AltHoldModeState::Flying:
        {
            //should only add pilot inputs to desired position if we are actually flying
            // HOODTECH MOD. -losh, 220831

            // always updated XY nudging if flying.
            g2.follow.update_pilot_nudge_xy(_pilot_vel,G_Dt);

            //determine if upward nudging should be allowed based on altitude.
            if (!allow_upward_nudging) {
                if (dist_vec.z > (Z_alt_up_nudge_allowed) ) { //
                    allow_upward_nudging = true ; //up nudging is a negative value of _desired_val.z by the way.
                    gcs().send_verbose_text(MAV_SEVERITY_ALERT,"up nudge now allowed.") ;
                }
            }

            // check pilot input is down/up and if its zero or non-zero, and if upward nudging is allowed
            if( (_pilot_vel.z > VEL_NUDGE_DEADBAND) || ((_pilot_vel.z<-VEL_NUDGE_DEADBAND) && allow_upward_nudging)) {
                //pilot is asking for down velocity or upward nudging is allowed and pilot wants up nudging.
                pilot_nudging = true;
                allow_upward_nudging = true; // if you nudged sucessfully in down direct, up has to be allowed now.

                g2.follow.update_pilot_nudge_z(_pilot_vel,G_Dt);

            }
            else if ((_pilot_vel.z<-VEL_NUDGE_DEADBAND) && !allow_upward_nudging){
                // pilot requesting up, but copter is under takeoff altitude limit still.
            }
            else {
                // pilot is not requesting nudging
                if(pilot_nudging==true){
                    // pilot had been nudging before, but now pilot is not.

                    // update local variable in AP_follow that holds nudge so that
                    // copter stays where it is currently.
                    if(!g2.follow.calc_pilot_nudge_based_on_current_location()){
                        //something went wrong trying to calculate the correct nudge.
                    };
                }
                else{
                    // this should be the case of pilot didnt request anything, and didnt request anything the iteration before that.
                    // so just get the accumulated pilot nudge and be done.
                }
                pilot_nudging = false;

            }

            // calc desired vel from target vel, target pos error, and pilot nudge requests.
            calc_des_vel( desired_velocity_neu_cms, dist_vec_offs, vel_of_target, _pilot_vel ) ;

            break;
        } //end follow_state==Flying
        default:
            break;
        }//end switch follow_state
    } //end if(get_target_dist_and_vel_ned)

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }
    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);

    ModeGuided::run();
}

uint32_t ModeFollow::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

int32_t ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeFollow::get_wp(Location &loc) const
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

/// @brief CAlculate desired velocities based on pil/target velocity and distance from target.
/// @param desired_velocity_neu_cms : output vels from controller/calculator
/// @param dist_vec_offs : dist vector between copter and where it should be
/// @param vel_of_target : velocity of target vehicle
/// @param pilot_vel : any pilot requested velocity.
void ModeFollow::calc_des_vel( Vector3f &desired_velocity_neu_cms, Vector3f &dist_vec_offs, Vector3f &vel_of_target, Vector3f &pilot_vel)
{    
    // add accumulated nudge to the offset vectors.
    dist_vec_offs += g2.follow.get_pilot_offset_nudges()  ; 

    // convert dist_vec_offs to cm in NEU
    const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);
    const float kp = g2.follow.get_pos_p().kP();    
    // this is diff than before, just going to limit things with pilot and distance, and then add in target later. maybe it will be better?
    // or maybe we'll have to add target back here.   
    desired_velocity_neu_cms.x = (pilot_vel.x* 100.0f) + (dist_vec_offs_neu.x * kp);
    desired_velocity_neu_cms.y = (pilot_vel.y* 100.0f) + (dist_vec_offs_neu.y * kp);
    // calculate desired velocity vector in cm/s in NEU, first add in pilot (desired) velocity then convert to cm/s. then add in distance error with kP.
    desired_velocity_neu_cms.z = (pilot_vel.z* -100.0f) + (dist_vec_offs_neu.z * kp); //negative vel because NEU

    // create horizontal desired velocity vector (required for slow down calculations)
    Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);

    // create horizontal unit vector towards target (required for slow down calculations)
    Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
    if (!dir_to_target_xy.is_zero()) dir_to_target_xy.normalize();

    // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
    const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
    copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
    // copy horizontal velocity limits back to 3d vector
    desired_velocity_neu_cms.xy() = desired_velocity_xy_cms;

    // limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
    const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
    desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

    // Add the target velocity baseline.
    desired_velocity_neu_cms.xy() += vel_of_target.xy() * 100.0f;
    desired_velocity_neu_cms.z += -vel_of_target.z * 100.0f;

    // scale desired velocity to stay within horizontal speed limit
    desired_velocity_neu_cms.xy().limit_length(pos_control->get_max_speed_xy_cms());

    // limit desired velocity to be between maximum climb and descent rates
    desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

    // limit the velocity for obstacle/fence avoidance
    copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

/*
    // calculate vehicle heading
    switch (g2.follow.get_yaw_behave()) {
        case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
            if (dist_vec.xy().length_squared() > 1.0) {
                yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
            float target_hdg = 0.0f;
            if (g2.follow.get_target_heading_deg(target_hdg)) {
                yaw_cd = target_hdg * 100.0f;
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
            if (desired_velocity_neu_cms.xy().length_squared() > (100.0 * 100.0)) {
                yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_neu_cms.xy());
                use_yaw = true;
            }
            break;
        }

        case AP_Follow::YAW_BEHAVE_NONE:
        default:
            // do nothing
            break;

    }
    */
}
#endif // MODE_FOLLOW_ENABLED == ENABLED
