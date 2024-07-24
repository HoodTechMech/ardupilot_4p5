#include "SRV_Channel/SRV_Channel.h"
//#include "../../ArduCopter/HT_Launch.h"

#define SERVO_PWM_SAFE          900    //pwm for un-release.  LOW NUMBERS ARE SAFE IN OUR (LITTLE) WORLD.
#define SERVO_PWM_UNSAFE        2100    //pwm for release
#define MAX_PWM                 2101
#define MIN_PWM                 500


class SRV_Hood_HAL {
public:

    typedef enum
    {
        k_lock      =   0,
        k_release   =   1
    } Hood_servo_func_t ; 

    // constructor
    SRV_Hood_HAL( Hood_servo_func_t ) ; 

    //eventually this has to be a single instance, that gets passed around.
    // how to do that? -losh 230522

    bool    servo_found( void) { return (_servo_index>=0) ;}
    bool    set_servo_unsafe(void)  ;
    bool    set_servo_safe(void)    ;
    bool    get_servo_safe(void)   ;

    void    set_pwm_unsafe( uint16_t unsafe_pwm ) ; 
    void    set_pwm_safe( uint16_t safe_pwm )   ;
    int8_t  get_servo_index( void ) { return _servo_index ; }

private:
    uint16_t    _pwm_unsafe  ;      // PWM value for making the servo be unsafe or open.
    uint16_t    _pwm_safe    ;      // PWM value for making the servo be safe or cloased.

    int8_t      _servo_index ;      // index in list of servos for this servo.

    int8_t      discover_servo( SRV_Channel::Aux_servo_function_t servo_func ) ; 
    bool        set_servo( uint16_t pwm ) ; 

} ;

