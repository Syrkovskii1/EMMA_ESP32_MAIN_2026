#include "EMMA_Stepper_Motor.h"

static uint8_t _step_pin;
static uint8_t _dir_pin;

static volatile long _current  = 0;
static volatile long _target   = 0;
static volatile bool _ready    = true;
static volatile bool _dir      = true;

// ── Параметры трапеции ────────────────
static volatile long  _total_steps  = 0;
static volatile long  _steps_done   = 0;
static volatile uint32_t _speed_min = 2000;
static volatile uint32_t _speed_max = 16000;
static volatile long  _accel_steps  = 100;
static volatile long  _decel_steps  = 130;
static volatile long  _real_accel   = 100;
static volatile long  _real_decel   = 130;

static hw_timer_t* _timer = NULL;

static bool _invert_dir = false;

// ── Расчёт интервала по текущей позиции ──
static uint32_t IRAM_ATTR calc_interval() {
    uint32_t speed;

    long steps_left = _total_steps - _steps_done;

    if (_steps_done < _real_accel) {
        // Разгон
        speed = _speed_min + (_speed_max - _speed_min) * _steps_done / _real_accel;
    } 
    else if (steps_left <= _real_decel) {
        // Торможение
        speed = _speed_min + (_speed_max - _speed_min) * steps_left / _real_decel;
    } 
    else {
        // Крейсер
        speed = _speed_max;
    }

    if (speed < _speed_min) speed = _speed_min;
    if (speed > _speed_max) speed = _speed_max;

    return 1000000 / speed / 2;
}

// ── ISR ───────────────────────────────
void IRAM_ATTR stepperISR() {
    static bool phase = false;

    if (_current == _target) {
        _ready = true;
        timerAlarmDisable(_timer);
        return;
    }

    // ЛОГИКА НАПРАВЛЕНИЯ УБРАНА ОТСЮДА (Она теперь в stepper_set_target)

    if (!phase) {
        digitalWrite(_step_pin, HIGH);
        phase = true;
    } else {
        digitalWrite(_step_pin, LOW);
        
        if (_dir) _current++;
        else      _current--;
        
        _steps_done++;

        // Обновляем скорость каждый шаг
        uint32_t interval = calc_interval();
        timerAlarmWrite(_timer, interval, true);

        phase = false;
    }
}

// ── Функции ───────────────────────────

void stepper_begin(uint8_t step_pin, uint8_t dir_pin,  bool invert_dir) {
    _step_pin = step_pin;
    _dir_pin  = dir_pin;
    _invert_dir = invert_dir;

    pinMode(_step_pin, OUTPUT);
    pinMode(_dir_pin,  OUTPUT);
    digitalWrite(_step_pin, LOW);
    
    // ИСПРАВЛЕНИЕ: Синхронизируем начальное состояние направления!
    _dir = true;
    digitalWrite(_dir_pin, (_dir ^ _invert_dir) ? HIGH : LOW);

    _timer = timerBegin(0, 80, true);
    timerAttachInterrupt(_timer, &stepperISR, true);
    timerAlarmWrite(_timer, 500, true);
    timerAlarmEnable(_timer);
    timerAlarmDisable(_timer);
}

void stepper_set_target(long target, uint32_t speed_max,
                        uint32_t speed_min,
                        long accel_steps, long decel_steps) {
    if (target == _current) {
        Serial.printf("[DEBUG STEPPER] Target %ld is equal to Current %ld. Ignored.\n", target, _current);
        return; // Protection against redundant triggers
    }

    _total_steps = abs(target - _current);
    _steps_done  = 0;
    _speed_max   = speed_max;
    _speed_min   = speed_min;
    _accel_steps = accel_steps;
    _decel_steps = decel_steps;

    Serial.printf("[DEBUG STEPPER] --- NEW TARGET ---\n");
    Serial.printf("[DEBUG STEPPER] Current: %ld | Target: %ld | Total steps to do: %ld\n", _current, target, _total_steps);

    // Protection for short movements
    if (_total_steps < accel_steps + decel_steps) {
        _real_accel = _total_steps / 2;
        _real_decel = _total_steps - _real_accel;
        Serial.printf("[DEBUG STEPPER] Short move detected. Adjusted Accel: %ld, Decel: %ld\n", _real_accel, _real_decel);
    } else {
        _real_accel = accel_steps;
        _real_decel = decel_steps;
    }

    // Set the physical direction pin IMMEDIATELY, before starting the timer
    _dir = (target > _current);
    digitalWrite(_dir_pin, (_dir ^ _invert_dir) ? HIGH : LOW);
    
    Serial.printf("[DEBUG STEPPER] Direction set to: %s (pin state: %s)\n", 
                  _dir ? "FORWARD (+)" : "REVERSE (-)", 
                  (_dir ^ _invert_dir) ? "HIGH" : "LOW");

    _ready  = false;
    _target = target;

    // Start with minimum speed
    uint32_t start_interval = 1000000 / speed_min / 2;
    
    timerAlarmWrite(_timer, start_interval, true);
    timerAlarmEnable(_timer);
}

void stepper_stop() {
    _target = _current;
    _ready  = true;
    timerAlarmDisable(_timer);
    digitalWrite(_step_pin, LOW); // Надежно опускаем пин шага
}

void stepper_reset() {
    timerAlarmDisable(_timer);
    _current     = 0;
    _target      = 0;
    _steps_done  = 0;
    _ready       = true;
    digitalWrite(_step_pin, LOW); // Надежно опускаем пин шага
}

bool stepper_is_ready() {
    return _ready;
}

long stepper_get_current() {
    return _current;
}

void stepper_set_position(long pos) {
    _current = pos;
    _target = pos;
}