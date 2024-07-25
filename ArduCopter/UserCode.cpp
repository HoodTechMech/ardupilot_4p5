#include "Copter.h"
#include "UserCode.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
	    hal.console->printf("USER INIT\n");
    // put your initialisation code here
    // this will be called once at start-up
  	mc40killCounter = 0;
	mc40KillCriteria = false;
	ch6KillArmStart = NEUTRAL; //init to neutral
	ch6HighCounter = 0; //initialize counter

	arm_switchPos = RC_Channel::AuxSwitchPos::HIGH;

	make_servos_safe();
	_lock_servo_state = eServo_state::NONE;
	_release_servo_state = eServo_state::NONE;

	RC7_SwitchPos = RC_Channel::AuxSwitchPos::MIDDLE;
	channel_rc7 = rc().channel(RC7_CH_INDEX) ;

	rc_chan_release = rc().channel(RELEASE_BUTTON_RC_INDEX);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // 10Hz code

	// keep track of the state of the ARM button.
	arm_switch_state_machine() ;

	// keep track of the state of the unlock/lock buttons
	lock_unlock_state_machine() ;

	// keep track of the state of the release button.
	release_state_machine();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // 1Hz code

	// give pilot a counter to when they should take off.
	takeoff_countdown_msg();

	#if MODE_LAUNCH_ENABLED==ENABLED
		g2.ht_launch.blower_off_check();
	#endif
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

// TAKEOFF_COUNTDOWN_MSG
// print out spin check time to takeoff msg at the right time.
void takeoff_countdown_msg( void )
{
	static bool prev_armed = false ;
	static bool prev_flying = false;
	static bool prev_notif = false;

	// HOODTECH MOD, losh 230109,
	// HOODTECH MOD, losh 230109,
	// this adds a spin check time, that allows taking off, but tries to get pilot not to
	// so that the Currawong ESCs can report back on whether or not they sucessfully
	// started or not.
	if( copter.get_arm_time() > 0 ) {
		uint32_t time_since_arm = (AP_HAL::millis() - copter.get_arm_time()) ;
		if( time_since_arm < 2*SPIN_CHK_TIME) {
			if( AP_Notify::flags.armed && !AP_Notify::flags.flying & !prev_flying){
				if( !prev_armed)
					{
						gcs().send_text(MAV_SEVERITY_WARNING,"ARMED: %ds to T/O", (int) roundf(SPIN_CHK_TIME/1000)) ;
						prev_notif = false ;
					}
				else {
					if( time_since_arm > SPIN_CHK_TIME ){
						if(!prev_notif){
						gcs().send_text(MAV_SEVERITY_WARNING, "OK TO T/O") ;
						prev_notif=true;
						}
					}
					else {
						gcs().send_text(MAV_SEVERITY_WARNING,"SPIN CHK: %ds to T/O", (int) roundf((SPIN_CHK_TIME/1000)- (time_since_arm/1000))) ;
					}
				}
			}
		}
	}
	prev_flying  = AP_Notify::flags.flying ;
	prev_armed = AP_Notify::flags.armed ;

}

void Copter::arm_switch_state_machine( void )
{

	if (!rc().channel(ARM_BUTTON_RC_INDEX)->read_3pos_switch(arm_switchPos))
	{
		arm_switchPos = RC_Channel::AuxSwitchPos::HIGH; //is this the safe option?
	}

	if ((ch6KillArmStart != NEUTRAL) && (arm_switchPos != RC_Channel::AuxSwitchPos::LOW))
	{
		if (ch6HighCounter > 2) {
			ch6KillArmStart = NEUTRAL;
			ch6HighCounter = 0;
		} else {
			ch6HighCounter++;
		}

	}

	if (!motors->armed()) {
		if (arm_switchPos == RC_Channel::AuxSwitchPos::LOW && ch6KillArmStart==0)
		{
			ch6KillArmStart = ARMING; //arming
		}
		return; //not armed, do nothing
	}


	if (arm_switchPos == RC_Channel::AuxSwitchPos::LOW && ch6KillArmStart == 0) {
		ch6KillArmStart = KILLING; //killing
	}

	switch (arm_switchPos) {
	case RC_Channel::AuxSwitchPos::LOW:
	// kill
		if (ch6KillArmStart != KILLING) //not in kill mode
		{
			break;
		}
		mc40killCounter++;
		if (mc40killCounter > 3){ //0.3 seconds, just over a quarter second
		//Stop the motor
			bool temp = ((channel_throttle->get_control_in() < 15) && (flightmode->get_alt_above_ground_cm() < 3000)); //throttle low, alt less than 30 meters

			if (mc40KillCriteria && temp){  //already killed
			}

			else if (temp){ //newly met criteria
				arming.disarm(AP_Arming::Method::AUXSWITCH);
				mc40KillCriteria = temp;
			}
			else{
				mc40KillCriteria = temp;
			}
			mc40killCounter = 0; //reset count
		}
	break;
	case RC_Channel::AuxSwitchPos::MIDDLE:
	case RC_Channel::AuxSwitchPos::HIGH:
        mc40killCounter = 0; //not currently killing so reset timers
		mc40KillCriteria = false;


		break;
	}
}

void Copter::lock_unlock_state_machine( void )
{
	// init servo state holder for this iter.
	static eServo_state new_lock_servo_state = eServo_state::UNSAFE ; //init unsafe so that it closes servos on first go.

	// attempt to read the switch position.
	bool lock_unlock_read = rc().channel(LOCK_UNLOCK_BUTTON_RC_INDEX)->read_3pos_switch(_lock_unlock_switchPos) ;

	//if position could not be read, assume the worse, and go safe.
	if( !lock_unlock_read ) {
		// set the state to be "SAFE"
		_lock_unlock_switchPos = RC_Channel::AuxSwitchPos::LOW; //is this the safe option?
	}

	// Update servo based on RC input.
	switch( _lock_unlock_switchPos){
		case RC_Channel::AuxSwitchPos::HIGH:
			new_lock_servo_state = eServo_state::UNSAFE ;
			break;
		default:
			new_lock_servo_state = eServo_state::SAFE ;
			break;
	}

	if( _lock_servo_state == eServo_state::NONE)
	{
		_lock_servo_state = new_lock_servo_state ;
	}
	if ( _lock_servo_state != new_lock_servo_state )
	{
		// update the servo state to the new one.
		_lock_servo_state = new_lock_servo_state ;
		// get a reference to the hood servo.
		lock_unlock_servo = new SRV_Hood_HAL( SRV_Hood_HAL::k_lock ) ;

		// update the state of the servo.
		if(update_servo_state( lock_unlock_servo, _lock_servo_state)){
			// delete the reference.
			delete lock_unlock_servo;
			HT_Launch *ht_launch = AP::ht_launch() ;

			// tell user:
			if(_lock_servo_state == eServo_state::UNSAFE) {
				if(ht_launch!=nullptr) {
					ht_launch->set_lockservo_state(COMMUNIC_UNSAFE) ;
					if(!motors->armed()) ht_launch->blower_on();
				}
				gcs().send_text(MAV_SEVERITY_WARNING, "UNLOCKED" ) ;
			} else {
				if(ht_launch!=nullptr){
					ht_launch->set_lockservo_state(COMMUNIC_SAFE) ;
					if(!motors->armed()) ht_launch->blower_off();
				}
				gcs().send_text(MAV_SEVERITY_WARNING, "LOCKED" ) ;
			}
		} else {
			gcs().send_text(MAV_SEVERITY_WARNING, "CHECK SERVO SETUP" ) ;
		}
	}
}

void Copter::release_state_machine( void )
{
	static eServo_state new_release_servo_state = eServo_state::UNSAFE ; //init unsafe so that it closes servos on first go.

	static int16_t jj = 0 ; //DEBUG?
	jj++; //DEBUG?

	// attempt to read the RC channel.
	if(!rc_chan_release->read_3pos_switch(_release_switchPos)) {
		_release_switchPos = RC_Channel::AuxSwitchPos::LOW;
	}

	// turn switch Pos into state.
	switch( _release_switchPos) {
		case RC_Channel::AuxSwitchPos::HIGH:

			// get a reference to the hood servo.
			lock_unlock_servo = new SRV_Hood_HAL( SRV_Hood_HAL::k_lock ) ;
			if( !lock_unlock_servo->get_servo_safe() ) {
				if((copter.get_mode() ==  (uint8_t) Mode::Number::ALT_HOLD)
					||(copter.get_mode() ==  (uint8_t) Mode::Number::LAUNCH)
					||(!AP_Notify::flags.armed)){
					new_release_servo_state = eServo_state::UNSAFE ;
				}
			} else {
				if(jj>10){ gcs().send_text(MAV_SEVERITY_WARNING, "UNLOCK BEFORE RELEASE" ) ;
				jj=0;}
			}
			delete lock_unlock_servo ;
			break;
		default:
			new_release_servo_state = eServo_state::SAFE ;
			break;
	}

	if( _release_servo_state == eServo_state::NONE) {
		_release_servo_state = new_release_servo_state ;
	}

	if( _release_servo_state != new_release_servo_state){

		gcs().send_text(MAV_SEVERITY_WARNING,"diff rel serv sts");
		if(copter.allow_release()) {
			_release_servo_state = new_release_servo_state;

			release_servo = new SRV_Hood_HAL( SRV_Hood_HAL::k_release ) ;

			// update the state of the servo.
			if(update_servo_state( release_servo, (eServo_state) _release_servo_state)){

				// delete the reference.
				delete release_servo;

				if(_release_servo_state == eServo_state::UNSAFE) {
					gcs().send_text(MAV_SEVERITY_WARNING, "RELEASE" ) ;
				}
			} else {
				gcs().send_text(MAV_SEVERITY_WARNING, "CHECK SERVO SETUP" ) ;
			}
		}
	}
}


bool Copter::allow_release()
{
	if(!motors->armed()) return true;

	switch(copter.flightmode->mode_number()){
		case Mode::Number::LAUNCH:		return (copter.mode_Launch.allow_release());
		case Mode::Number::ALT_HOLD:	return true;
		default: 						return false;
	}
	return false;
}

// set hood servos to SAFE condition.
bool make_servos_safe( void )
{
	release_servo = new SRV_Hood_HAL( SRV_Hood_HAL::k_release ) ;
	bool release_servo_safe = release_servo->set_servo_safe();
	delete release_servo ;

	lock_unlock_servo = new SRV_Hood_HAL( SRV_Hood_HAL::k_lock ) ;
	bool lock_servo_safe = lock_unlock_servo->set_servo_safe();
	delete lock_unlock_servo ;

	return( release_servo_safe && lock_servo_safe) ;
}

// update the state of a hood lock/release servo.
bool Copter::update_servo_state( SRV_Hood_HAL* servo, eServo_state state )
{
			// move servo accordingly.
		switch(state)
		{
			case eServo_state::UNSAFE:
				return(servo->set_servo_unsafe());
				break;
			default:
				return(servo->set_servo_safe());
				break ;

		}
		return false;


}

// determine RC7s input state.
bool Copter::rc7_state_machine(void)
{
	// ---------------------------------------------------------------------------------
	// TOGGLE AUTO MODES STATE MACHINE ---------------------------------------------
	// read switch position of ch7 button (used to be 1/2 pwr mode.)
	if(!channel_rc7->read_3pos_switch(RC7_SwitchPos)) {
		RC7_SwitchPos = RC_Channel::AuxSwitchPos::MIDDLE;
	}

	if(true) {
		switch( RC7_SwitchPos){
			case RC_Channel::AuxSwitchPos::HIGH:
				return true;
			case RC_Channel::AuxSwitchPos::LOW:
				return false;
			default:
				return false;
		}
	}
	// ---------------------------------------------------------------------------------
}
