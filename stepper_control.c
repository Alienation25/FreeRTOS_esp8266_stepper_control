

#include "stepper_control.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>




void MyClass_Init(struct stepper_control *stepper_in, short dir_pin, short step_pin,short enable_pin, float rpm ,short accel, short decel, short microsteps,short steps ,short enable_active_state ) 
{
    stepper_in->dir_pin = dir_pin;
    stepper_in->step_pin = step_pin;
    stepper_in->enable_pin =enable_pin;
    stepper_in->rpm =rpm;
    stepper_in->accel = accel;
    stepper_in->decel = decel;
    stepper_in->microsteps =microsteps;
    stepper_in->steps = steps;
    stepper_in->motor_steps =steps;
    stepper_in->enable_active_state =enable_active_state;

    stepper_in->dir_state =0;
    stepper_in->steps_to_cruise =0;
    stepper_in->steps_remaining =0;
    stepper_in->steps_to_brake =0;
    stepper_in->step_pulse=0;
    stepper_in->cruise_step_pulse=0;
    stepper_in->step_count =0;
    stepper_in->rest =0;
    stepper_in->last_action_end =0;
    stepper_in->next_action_interval =0;

   
    stepper_in->Mode =1;
}


void enable (struct stepper_control *stepper_in,short steps)
{
    stepper_in->enable_active_state = steps;
}


void delayMicros(unsigned long delay_us, unsigned long start_us )
{
    if (delay_us) {
        if (!start_us) {
            start_us = esp_timer_get_time();
        }
        if (delay_us > MIN_YIELD_MICROS) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        // See https://www.gammon.com.au/millis
        while ((esp_timer_get_time() - start_us) < delay_us);
    }
}

void begin_pin(struct stepper_control *stepper_in)
{
        gpio_config(& (gpio_config_t) {
            .pin_bit_mask = (1ULL << stepper_in->dir_pin),
            .mode = 2,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        });
        gpio_config(& (gpio_config_t) {
            .pin_bit_mask = (1ULL << stepper_in->step_pin),
            .mode = 2,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        });
            gpio_config(& (gpio_config_t) {
            .pin_bit_mask = (1ULL << stepper_in->enable_pin),
            .mode = 2,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        });
}



short getCurrentState(struct stepper_control *stepper_in)
{
    short state;
    if (stepper_in->steps_remaining <= 0){
        state = STOPPED;
    } else {
        if (stepper_in->steps_remaining <= stepper_in->steps_to_brake){
            state =  DECELERATING;
        } else if (stepper_in->step_count <= stepper_in->steps_to_cruise){
            state = ACCELERATING;
        } else {
            state = CRUISING;
        }
    }
    return state;
}





void calcStepPulse(struct stepper_control *stepper_in)
{
    if (stepper_in->steps_remaining <= 0){  // этого не должно происходить, но позволяет избежать странных расчетов
        return;
    }
    stepper_in->steps_remaining--;
    stepper_in->step_count++;
    if (stepper_in->Mode == LINEAR_SPEED){
        switch (getCurrentState(stepper_in)){
        case ACCELERATING:
            if (stepper_in->step_count < stepper_in->steps_to_cruise){
                stepper_in->step_pulse = stepper_in->step_pulse - (2*stepper_in->step_pulse+stepper_in->rest)/(4*stepper_in->step_count+1);
                stepper_in->rest = (stepper_in->step_count < stepper_in->steps_to_cruise) ? (2*stepper_in->step_pulse+stepper_in->rest) % (4*stepper_in->step_count+1) : 0;
            } else {
                // The series approximates target, set the final value to what it should be instead
                stepper_in->step_pulse = stepper_in->cruise_step_pulse;
            }
            break;

        case DECELERATING:
            stepper_in->step_pulse = stepper_in->step_pulse - (2*stepper_in->step_pulse+stepper_in->rest)/(-4*stepper_in->steps_remaining+1);
            stepper_in->rest = (2*stepper_in->step_pulse+stepper_in->rest) % (-4*stepper_in->steps_remaining+1);
            break;

        default:
            break; // no speed changes
        }
    }
}



long nextAction(struct stepper_control *stepper_in)
{
    if (stepper_in->steps_remaining > 0){
        //если оставшоеся количество шагов не равно 0 то начинаем работу 
        delayMicros(stepper_in->next_action_interval, stepper_in->last_action_end); 
        //изначально равен  0 задаёться в структуре в privat
        //next_action_interval
        //приравниваться к нулю в переменой startMove
        //last_action_end
        gpio_set_level(stepper_in->dir_pin, stepper_in->dir_state);
        //задаёться в dir_pin в функции begin
        //направление движения dir_state находиться startMove
        gpio_set_level(stepper_in->step_pin, 1);
        //тоже задаёться в begin
        unsigned m = esp_timer_get_time();
        unsigned long pulse = stepper_in->step_pulse; // 
        calcStepPulse(stepper_in);
        // We should pull HIGH for at least 1-2us (step_high_min)
        delayMicros(step_high_min, 0);
        gpio_set_level(stepper_in->step_pin, 0);
        // account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
        stepper_in->last_action_end = esp_timer_get_time();
        m = stepper_in->last_action_end - m;
        stepper_in->next_action_interval = (pulse > m) ? pulse - m : 1;
    } else {
        // end of move
        stepper_in->last_action_end = 0;
        stepper_in->next_action_interval = 0; 
        //занчение по которому происходит остановка работы
    }
    return stepper_in->next_action_interval;
}


void startMove(struct stepper_control *stepper_in,long steps, long time)
{
    float speed;
    // set up new move
    stepper_in->dir_state = (steps >= 0) ? 1 : 0;
    //напраление движения 
    stepper_in->last_action_end = 0;
    /*
     это переменная, которая содержит время в 
     микросекундах, когда последнее действие мотора 
     было завершено-> Она используется для расчета интервала времени
      между двумя последовательными действиями мотора->
    */
    stepper_in->steps_remaining = labs(steps);
     /*
     steps_remaining - это переменная, которая содержит 
     оставшееся количество шагов, которые должен сделать мотор.
     Она используется для определения того, когда движение должно быть завершено.
    */
    stepper_in->step_count = 0;
    stepper_in->rest = 0;
    switch (stepper_in->Mode){
    case LINEAR_SPEED:
        // speed is in [steps/s]
        // 
        speed = stepper_in->rpm * stepper_in->motor_steps / 60;
        //скорость  = (оборотов в минуту )*(количество шагов, которое делает двигатель за один оборот)/60

        if (time > 0){
            // Calculate a new speed to finish in the time requested
            float t = time / (1e+6);                  // convert to seconds
            float d = stepper_in->steps_remaining / stepper_in->microsteps;   // convert to full steps
            float a2 = 1.0 / stepper_in->accel + 1.0 / stepper_in->decel;
            float sqrt_candidate = t*t - 2 * a2 * d;  // in √b^2-4ac
            if (sqrt_candidate >= 0){
                speed = fmin(speed, (t - (float)sqrt(sqrt_candidate)) / a2);
            };
        }

        stepper_in->steps_to_cruise = stepper_in->microsteps * (speed * speed / (2 * stepper_in->accel));
         /*
        Эта строка кода вычисляет количество микрошагов, 
        которые требуются, чтобы достигнуть максимальной скорости двигателя
        microsteps - микрошагов
        steps_to_cruise
        */
        stepper_in->steps_to_brake = stepper_in->steps_to_cruise * stepper_in->accel / stepper_in->decel;
        /*
        - количество шагов, необходимых для достижения скорости круиза
        steps_to_brake
        */
        if (stepper_in->steps_remaining < stepper_in->steps_to_cruise + stepper_in->steps_to_brake){
          // не может достичь максимальной скорости, нужно затормозить раньше
            stepper_in->steps_to_cruise = stepper_in->steps_remaining * stepper_in->decel / (stepper_in->accel + stepper_in->decel);
            //количество шагов для разгона
            stepper_in->steps_to_brake = stepper_in->steps_remaining - stepper_in->steps_to_cruise;
            //количество шагов для торможения 
        }
        // Начальный импульс (c0), включая коэффициент коррекции ошибки 0,676 [мкс]
        stepper_in->step_pulse = (1e+6)*0.676*sqrt(2.0f/stepper_in->accel/stepper_in->microsteps);
        /*
          это переменная, которая содержит длительность в микросекундах одного шага мотора на т
          екущей скорости и ускорении. Она используется для определения интервала времени между
           двумя последовательными шагами мотора.
        */
        // Сохранить время круиза, так как позже у нас больше не будет расчетной целевой скорости
        stepper_in->cruise_step_pulse = 1e+6 / speed / stepper_in->microsteps;
        /*
        это переменная, которая содержит длительность в микросекундах одного шага мотора на круиз-скорости.
        Она используется для определения интервала времени между двумя последовательными шагами мотора на круиз-скорости.
        */
        break;

    case CONSTANT_SPEED:
    default:
        stepper_in->steps_to_cruise = 0;
        stepper_in->steps_to_brake = 0;
        stepper_in->step_pulse = stepper_in->cruise_step_pulse = STEP_PULSE(stepper_in->motor_steps, stepper_in->microsteps, stepper_in->rpm);
        if (time > stepper_in->steps_remaining * stepper_in->step_pulse){
            stepper_in->step_pulse = (float)time / stepper_in->steps_remaining;
        }
    }
}



void move(struct stepper_control *stepper_in,long steps){
    startMove(stepper_in,steps,0);
    while (nextAction(stepper_in));
}