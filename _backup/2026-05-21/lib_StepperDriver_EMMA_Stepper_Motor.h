#pragma once
#include <Arduino.h>

void stepper_begin(uint8_t step_pin, uint8_t dir_pin, bool invert_dir = false);


// speed_max  — крейсерская скорость шаг/сек
// speed_min  — скорость старта и финиша
// accel_steps — шагов на разгон
// decel_steps — шагов на торможение
void stepper_set_target(long target,
                        uint32_t speed_max  = 18000,
                        uint32_t speed_min  = 2000,
                        long accel_steps    = 200,
                        long decel_steps    = 200);

void stepper_stop();
void stepper_reset();
bool stepper_is_ready();
long stepper_get_current();

void stepper_set_position(long pos);