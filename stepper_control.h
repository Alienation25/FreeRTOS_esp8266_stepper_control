// stepper_control.h

#ifndef STEPPER_CONTROL_HEADER_GUARD
#define STEPPER_CONTROL_HEADER_GUARD

#define MIN_YIELD_MICROS 50
#define STEP_PULSE(steps, microsteps, rpm) (60.0*1000000L/steps/microsteps/rpm)

#define LINEAR_SPEED  1  //1
#define CONSTANT_SPEED 2 //2


static const int    step_high_min = 1; //1//const
static const int    step_low_min = 1;  //1//const


typedef enum {
    STOPPED,              /*!< UART data event*/
    ACCELERATING,       /*!< UART RX buffer full event*/
    CRUISING,          /*!< UART FIFO overflow event*/
    DECELERATING,         /*!< UART RX frame error event*/
} stepper_status;

 struct  stepper_control
{
    short steps ;//200 
    short motor_steps ;//200
    short dir_pin ;//0
    short step_pin ;//1
    short enable_pin ;//2
    float rpm ; //5
    short microsteps ;//16
    short Mode ;//1
    short accel ;//1000   
    short decel ; //1000
    short enable_active_state ; //0
    short dir_state ;//0
    short steps_to_cruise;//0
    short steps_remaining;//0
    short steps_to_brake;//0
    short step_pulse;//0
    short cruise_step_pulse;//0
    short step_count;//0
    long  rest ;//0
    unsigned long last_action_end ;//0
    unsigned long next_action_interval ;//0

   
} ;

void delayMicros(unsigned long delay_us, unsigned long start_us );
void begin_pin(struct stepper_control *stepper_in);
void calcStepPulse(struct stepper_control *stepper_in);
short getCurrentState(struct stepper_control *stepper_in);
void startMove(struct stepper_control *stepper_in,long steps, long time);
long nextAction(struct stepper_control *stepper_in);
void move(struct stepper_control *stepper_in,long steps);
void enable (struct stepper_control *stepper_in,short steps);
void MyClass_Init(struct stepper_control *stepper_in, short dir_pin, short step_pin,short enable_pin, float rpm ,short accel, short decel, short microsteps,short steps ,short enable_active_state );


#endif
