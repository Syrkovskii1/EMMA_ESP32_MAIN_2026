#include "EMMA_Fingers.h"
#include "EMMA_Stepper_Motor.h"

static int   _active_finger = -1;
static long  _delta         = 0;
static long  _start_pos     = 0; 
static bool  _moving        = false;
static int   _notes_count   = 0;
static bool  _press         = true; 

static unsigned long _lift_timer = 0;
static bool _waiting_lift = false;
const unsigned long SERVO_LIFT_DELAY = 150; // ms (Increase if servo is slow)

void fingers_init(long* notes, int notes_count, long* fingers, int pulses_per_mm) {
    _notes_count = notes_count;
    Serial.printf("\n[DEBUG FINGER] --- INIT STARTED ---\n");
    Serial.printf("[DEBUG FINGER] Pulses per mm: %d | Notes count: %d\n", pulses_per_mm, notes_count);
    
    // Convert mm to motor steps
    for (int i = 0; i < notes_count; i++) {
        notes[i] *= pulses_per_mm;
        Serial.printf("[DEBUG FINGER] Note %d mapped to pos: %ld\n", i + 1, notes[i]);
    }
    
    // Initial finger positions (linked to specific notes by default)
    fingers[0] = notes[1];
    fingers[1] = notes[2];
    fingers[2] = notes[4];
    fingers[3] = notes[6];

    for(int i = 0; i < 4; i++) {
        Serial.printf("[DEBUG FINGER] Finger %d starts at pos: %ld\n", i, fingers[i]);
    }
    Serial.printf("[DEBUG FINGER] --- INIT DONE ---\n\n");
}

int find_nearest_finger(long target, long* fingers) {
    int  best_index = 0;
    long best_diff  = abs(target - fingers[0]);
    
    Serial.printf("[DEBUG FINGER] Searching nearest finger for target pos: %ld\n", target);
    Serial.printf("[DEBUG FINGER] -> Finger 0 pos: %ld | diff: %ld\n", fingers[0], best_diff);

    for (int i = 1; i < 4; i++) {
        long diff = abs(target - fingers[i]);
        Serial.printf("[DEBUG FINGER] -> Finger %d pos: %ld | diff: %ld\n", i, fingers[i], diff);
        if (diff < best_diff) {
            best_diff  = diff;
            best_index = i;
        }
    }
    Serial.printf("[DEBUG FINGER] => SELECTED FINGER: %d (Best diff: %ld)\n", best_index, best_diff);
    return best_index;
}

void set_note(int nota, long* notes, long* fingers, bool press) {
    if (nota < 1 || nota > _notes_count) {
        Serial.printf("[DEBUG ERROR] Note %d is out of bounds!\n", nota);
        return;
    }
    
    // Prevent overriding commands if busy or waiting for servo
    if (_moving || _waiting_lift) {
        Serial.println("[DEBUG WARNING] set_note ignored. System is busy moving or lifting.");
        return; 
    }

    Serial.printf("\n[DEBUG FINGER] --- NEW COMMAND: set_note --- \n");
    Serial.printf("[DEBUG FINGER] Requested nota: %d | Press mode: %d\n", nota, press);

    long target    = notes[nota - 1];
    int  finger    = find_nearest_finger(target, fingers);
    
    _delta         = target - fingers[finger];
    _active_finger = finger;
    _start_pos     = stepper_get_current();
    _press         = press; 

    Serial.printf("[DEBUG FINGER] Target POS: %ld\n", target);
    Serial.printf("[DEBUG FINGER] Current Stepper POS (_start_pos): %ld\n", _start_pos);
    Serial.printf("[DEBUG FINGER] Calculated DELTA: %ld (Target %ld - FingerPos %ld)\n", _delta, target, fingers[finger]);

    activate_finger(-1); 

    // Start the delay timer instead of moving the stepper instantly
    _waiting_lift = true;
    _lift_timer = millis();
    Serial.println("[DEBUG FINGER] Lifting fingers... Waiting for servo delay.");
}

void fingers_update(long* fingers) {
    // Wait for physical servo movement before starting stepper
    if (_waiting_lift) {
        if (millis() - _lift_timer >= SERVO_LIFT_DELAY) {
            _waiting_lift = false;
            long calculated_target = _start_pos + _delta;
            Serial.printf("[DEBUG FINGER] Servo delay finished. Sending stepper to: %ld\n", calculated_target);
            stepper_set_target(calculated_target);
            _moving = true;
        }
        return; // Skip the rest until timer completes
    }

    if (!_moving) return;

    // Wait until motor finishes movement
    if (stepper_is_ready()) {
        long real_shift = stepper_get_current() - _start_pos;
        Serial.printf("\n[DEBUG FINGER] Stepper reached target! Real shift was: %ld\n", real_shift);
        Serial.println("[DEBUG FINGER] Updating finger positions:");

        for (int i = 0; i < 4; i++) {
            fingers[i] += real_shift;
            Serial.printf("  -> Finger %d new absolute pos: %ld\n", i, fingers[i]);
        }

        if (_press) {
            activate_finger(_active_finger); 
        } else {
            activate_finger(-1); 
        }

        String names[] = {"Index", "Middle", "Ring", "Pinky"};
        Serial.print("[FINGER ACT] ");
        if (_press) {
            Serial.print(names[_active_finger]);
            Serial.println(" PRESSED");
        } else {
            Serial.println("ALL LIFTED (carriage moved only)");
        }

        _moving = false; 
        Serial.println("[DEBUG FINGER] --- MOVEMENT COMPLETE ---\n");
    }
}

int get_active_finger() {
    return _active_finger;
}

// Check if fingers are currently moving or waiting for servo delay
bool fingers_are_busy() {
    return _moving || _waiting_lift;
}