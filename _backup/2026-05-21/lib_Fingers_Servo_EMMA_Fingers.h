#pragma once
#include <Arduino.h>

void fingers_init(long* notes, int notes_count, long* fingers, int pulses_per_mm);
int  find_nearest_finger(long target, long* fingers);
void set_note(int nota, long* notes, long* fingers, bool press = true);
void fingers_update(long* fingers);

// Эту функцию пишешь в main.cpp
extern void activate_finger(int index);

int get_active_finger();
bool fingers_are_busy();