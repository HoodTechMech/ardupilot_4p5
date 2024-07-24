// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

enum mc40KillArmStart{
NEUTRAL,
ARMING,
KILLING,
};

enum eServo_state{
    SAFE,
    UNSAFE,
    NONE,
} ;

//mc40 variables
int8_t mc40killCounter;
bool mc40KillCriteria;
int8_t mc40RampPosition;
mc40KillArmStart ch6KillArmStart;
int8_t ch6HighCounter;
int16_t armedTimer;
RC_Channel::AuxSwitchPos    arm_switchPos;
RC_Channel::AuxSwitchPos    _lock_unlock_switchPos;
RC_Channel::AuxSwitchPos    _release_switchPos;

// servo instances.
SRV_Hood_HAL* lock_unlock_servo ;
SRV_Hood_HAL* release_servo ;

eServo_state _lock_servo_state = eServo_state::NONE;
eServo_state _release_servo_state = eServo_state::NONE;

RC_Channel::AuxSwitchPos RC7_SwitchPos ;

RC_Channel *channel_rc7 ;


#endif  // USERHOOK_VARIABLES


