/* ════════════════════════════════════════════════════════════════
 * EMMA Stepper Motor (Robust LUT Edition)
 * ----------------------------------------------------------------
 * Оптимизированный движок. Кэширует только разгон и торможение.
 * Совместим с огромными шагами калибровки (999999). Zero-division в ISR.
 * ════════════════════════════════════════════════════════════════ */

#include "EMMA_Stepper_Motor.h"
#include "soc/gpio_reg.h"   // Прямой доступ к регистрам GPIO
#include <math.h>           // Для sqrtf

#define MAX_STAGE_STEPS 1024  // Макс. шагов на разгон или торможение с запасом

static uint8_t _step_pin;
static uint8_t _dir_pin;
static bool    _invert_dir = false;

static volatile uint32_t _steps_to_do = 0;       
static volatile uint32_t _steps_done  = 0;       
static volatile bool     _ready       = true;    
static volatile bool     _step_phase  = false;   

// Раздельные таблицы для разгона и торможения
static uint32_t _accel_intervals[MAX_STAGE_STEPS]; 
static uint32_t _decel_intervals[MAX_STAGE_STEPS]; 
static uint32_t _cruise_interval = 0;

static volatile uint32_t _real_accel = 0;
static volatile uint32_t _real_decel = 0;

static hw_timer_t* _timer = NULL;
static portMUX_TYPE _stepper_mux = portMUX_INITIALIZER_UNLOCKED;

/* ─── Быстрое управление пином STEP (12 нс) ──────────────────── */
static inline void IRAM_ATTR fast_step_high() {
    if (_step_pin < 32) GPIO.out_w1ts = (1 << _step_pin);
    else                GPIO.out1_w1ts.val = (1 << (_step_pin - 32));
}

static inline void IRAM_ATTR fast_step_low() {
    if (_step_pin < 32) GPIO.out_w1tc = (1 << _step_pin);
    else                GPIO.out1_w1tc.val = (1 << (_step_pin - 32));
}

/* ════════════════════════════════════════════════════════════════
 * stepperISR
 * ════════════════════════════════════════════════════════════════ */
void IRAM_ATTR stepperISR() {
    if (_steps_done >= _steps_to_do) {
        timerAlarmDisable(_timer);
        fast_step_low();
        _ready = true;
        return;
    }

    if (!_step_phase) {
        fast_step_high();
        _step_phase = true;
    } else {
        fast_step_low();
        _step_phase = false;

        _steps_done++;

        if (_steps_done < _steps_to_do) {
            uint32_t steps_left = _steps_to_do - _steps_done;
            uint32_t interval;

            if (_steps_done < _real_accel) {
                // Мы в фазе разгона — берем интервал из таблицы разгона
                interval = _accel_intervals[_steps_done];
            } else if (steps_left <= _real_decel) {
                // Мы в фазе торможения — берем интервал из таблицы торможения
                interval = _decel_intervals[_real_decel - steps_left];
            } else {
                // Мы на крейсерской полке — берем константу
                interval = _cruise_interval;
            }

            timerAlarmWrite(_timer, interval, true);
        }
    }
}

void stepper_begin(uint8_t step_pin, uint8_t dir_pin, bool invert_dir) {
    _step_pin   = step_pin;
    _dir_pin    = dir_pin;
    _invert_dir = invert_dir;

    pinMode(_step_pin, OUTPUT);
    pinMode(_dir_pin,  OUTPUT);
    digitalWrite(_step_pin, LOW);
    digitalWrite(_dir_pin,  LOW);

    _timer = timerBegin(0, 80, true); // 1 такт = 1 мкс
    timerAttachInterrupt(_timer, &stepperISR, true);
}

void stepper_move(uint32_t steps, bool dir, uint32_t speed_max, uint32_t speed_min, uint32_t accel, uint32_t decel) {
    if (steps == 0) return;

    if (speed_min == 0)        speed_min = 1;
    if (speed_max < speed_min) speed_max = speed_min;

    if (!_ready) return;

    // Расчет и защита от выхода за рамки буфера стадий
    _real_accel = accel;
    _real_decel = decel;

    if (steps < accel + decel) {
        _real_accel = steps / 2;
        _real_decel = steps - _real_accel;
    }

    if (_real_accel > MAX_STAGE_STEPS) _real_accel = MAX_STAGE_STEPS;
    if (_real_decel > MAX_STAGE_STEPS) _real_decel = MAX_STAGE_STEPS;
    if (_real_accel == 0) _real_accel = 1;
    if (_real_decel == 0) _real_decel = 1;

    // Расчет физического профиля v(t)
    float speed_min_sq = (float)speed_min * speed_min;
    float speed_max_sq = (float)speed_max * speed_max;
    
    float a = (speed_max_sq - speed_min_sq) / (2.0f * (float)accel);
    float current_speed_max_sq = speed_min_sq + 2.0f * a * (float)_real_accel;
    float current_speed_max = sqrtf(current_speed_max_sq);
    if (current_speed_max > speed_max) current_speed_max = (float)speed_max;

    float eff_a = (current_speed_max_sq - speed_min_sq) / (2.0f * (float)_real_accel);
    float eff_d = (current_speed_max_sq - speed_min_sq) / (2.0f * (float)_real_decel);

    // 1. Крейсерский интервал (полка)
    _cruise_interval = (uint32_t)(1000000.0f / current_speed_max / 2.0f);

    // 2. Заполняем таблицу разгона
    for (uint32_t i = 0; i < _real_accel; i++) {
        float speed = sqrtf(speed_min_sq + 2.0f * eff_a * (float)i);
        if (speed < speed_min) speed = speed_min;
        _accel_intervals[i] = (uint32_t)(1000000.0f / speed / 2.0f);
    }

    // 3. Заполняем таблицу торможения
    for (uint32_t i = 0; i < _real_decel; i++) {
        uint32_t steps_left_formula = _real_decel - i;
        float speed = sqrtf(speed_min_sq + 2.0f * eff_d * (float)steps_left_formula);
        if (speed < speed_min) speed = speed_min;
        _decel_intervals[i] = (uint32_t)(1000000.0f / speed / 2.0f);
    }

    // Безопасный пуск в критической секции
    portENTER_CRITICAL(&_stepper_mux);

    if (_invert_dir) {
        digitalWrite(_dir_pin, dir ? LOW : HIGH);
    } else {
        digitalWrite(_dir_pin, dir ? HIGH : LOW);
    }

    _steps_to_do = steps;
    _steps_done  = 0;
    _step_phase  = false;
    _ready       = false;
    
    digitalWrite(_step_pin, LOW);

    timerAlarmWrite(_timer, _accel_intervals[0], true);
    timerAlarmEnable(_timer);

    portEXIT_CRITICAL(&_stepper_mux);
}

void stepper_stop() {
    timerAlarmDisable(_timer);
    portENTER_CRITICAL(&_stepper_mux);
    _steps_to_do = _steps_done;
    _ready       = true;
    portEXIT_CRITICAL(&_stepper_mux);
    fast_step_low();
}

bool stepper_is_ready() {
    return _ready;
}

uint32_t stepper_get_steps_left() {
    if (_ready) return 0;
    return _steps_to_do - _steps_done;
}

uint32_t stepper_get_total_steps() {
    return _steps_to_do;
}