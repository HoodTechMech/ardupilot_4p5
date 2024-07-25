#include "UserVariables.h"
//#include "HT_AutoModes.h"


#define ARM_BUTTON_RC_INDEX             5
#define RELEASE_BUTTON_RC_INDEX         7
#define LOCK_UNLOCK_BUTTON_RC_INDEX     8
#define RC7_CH_INDEX                    6 //ch7 -1 for indexing.
#define SPIN_CHK_TIME                   8000


// func that generates the countdown to T/O
void takeoff_countdown_msg( void ) ;

bool make_servos_safe( void ) ;

// func that generates the countdown to T/O
void takeoff_countdown_msg( void ) ;