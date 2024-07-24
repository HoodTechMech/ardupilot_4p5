#include "SRV_Hood_HAL.h"


/*
constructor
*/
SRV_Hood_HAL::SRV_Hood_HAL( Hood_servo_func_t servo_type )
{

    switch( servo_type ) {
        case k_lock:
        {
            _servo_index = discover_servo( SRV_Channel::k_HOOD_Lock_Servo ) ; 
            break ; 
        }
        case k_release:
        {
            _servo_index = discover_servo( SRV_Channel::k_HOOD_Release_Servo ) ; 
            break ;
        }
        default:
        _servo_index = -23;
        break;
    }

    if(_servo_index>=0){
        SRV_Channel *servo = SRV_Channels::srv_channel(_servo_index);
        if (servo != nullptr) {
            //set safe/unsafe pwm values based on parameters so flares3 can have its param cfg'd servos.
            set_pwm_unsafe( servo->get_limit_pwm( SRV_Channel::Limit::MAX ) ) ; 
            set_pwm_safe(   servo->get_limit_pwm( SRV_Channel::Limit::MIN ) ) ;
        }
    }
}




int8_t SRV_Hood_HAL::discover_servo(SRV_Channel::Aux_servo_function_t servo_func)
{
    for( int8_t servo_i=0; servo_i < NUM_SERVO_CHANNELS; servo_i++ )
    {
        SRV_Channel *servo = SRV_Channels::srv_channel(servo_i);
        if (servo != nullptr) {
            if( servo->get_function() == servo_func){
                return servo_i ; 
            } 
        }         
    }
    return -1;

}


// provides access to direct servo control 
bool SRV_Hood_HAL::set_servo( uint16_t pwm )
{
    // exgtra check that dont try to set the PWM of a motor.
    if(_servo_index<8) { return false; }

    SRV_Channel *servo = SRV_Channels::srv_channel(_servo_index);
    if (servo == nullptr) { return false; }
   
    //set the servo PWM. 
    servo->set_output_pwm(pwm) ; 
    servo->ignore_small_rcin_changes() ;
    return true;
}

//
// SRV_HOOD_HAL: set servo to UNSAFE condition. 
// for us, this means "open" the servo.
//
bool SRV_Hood_HAL::set_servo_unsafe( void )
{
    if( _pwm_unsafe != 0) {
        return set_servo( _pwm_unsafe )  ;
    }
    return false;
}

// SET_SERVO_SAFE
// changes state of servo to the safe PWM setting.
bool SRV_Hood_HAL::set_servo_safe( void )
{
    if( _pwm_safe != 0) {
        return set_servo( _pwm_safe ) ; 
    }
    return false;
}

// SET_PWM_UNSAFE
// sets the PWM value for UNSAFE or OPEN servo condition.
void SRV_Hood_HAL::set_pwm_unsafe( uint16_t incoming_pwm ) 
{
    if((incoming_pwm<MAX_PWM)&&(incoming_pwm>MIN_PWM))
    {
        _pwm_unsafe = incoming_pwm ; 
    }
}

// SET_PWM_SAFE
// sets the PWM value for SAFE or CLOSED servo condition.
void SRV_Hood_HAL::set_pwm_safe( uint16_t incoming_pwm ) 
{
    if((incoming_pwm<MAX_PWM)&&(incoming_pwm>MIN_PWM))
    {
        _pwm_safe = incoming_pwm ; 
    }
}

// GET_SERVO_SAFE
// returns true if current PWM equals safe PWM.
bool SRV_Hood_HAL::get_servo_safe() 
{

    SRV_Channel *servo = SRV_Channels::srv_channel(_servo_index);
    if (servo == nullptr) { return false; }
   
    //get the servo PWM. 
    uint16_t servoPWM = servo->get_output_pwm() ; 

    if (servoPWM==_pwm_safe)    return true;
    else                        return false;

}