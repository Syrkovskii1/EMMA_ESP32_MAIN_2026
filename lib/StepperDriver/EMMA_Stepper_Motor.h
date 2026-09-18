#pragma once
#include <Arduino.h>

void stepper_begin(uint8_t step_pin, uint8_t dir_pin, bool invert_dir = false);

// Move the stepper a specific number of steps in a given direction
// speed_max   - cruise speed (steps/sec)
// speed_min   - start/stop speed (steps/sec)
// accel_steps - steps for acceleration
// decel_steps - steps for deceleration
void stepper_move(uint32_t steps_to_do, bool dir,
                  uint32_t speed_max   = 18000,
                  uint32_t speed_min   = 2000,
                  uint32_t accel_steps = 200,
                  uint32_t decel_steps = 200);

// менять не тут параметры а в EMMA fingers 96 строчка для ноты 

void stepper_stop();
bool stepper_is_ready();
uint32_t stepper_get_steps_left();
uint32_t stepper_get_total_steps();

// We no longer need stepper_set_position() or stepper_get_current()
// The stepper is now a "dumb" executor.