#pragma once
#include <Arduino.h>

void fingers_init(long* notes, int notes_count, int pulses_per_mm);
void fingers_set_home(long* notes);

void set_note(int nota, long* notes);

void fingers_update();
bool fingers_are_busy();
int  get_active_finger();


extern void activate_finger(int index);