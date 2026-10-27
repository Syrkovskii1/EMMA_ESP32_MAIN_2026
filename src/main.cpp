
#include <Arduino.h>
#include <math.h>
#include "EMMA_Stepper_Motor.h"
#include "EMMA_Fingers.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "BluetoothSerial.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>



#include <Preferences.h>

Preferences prefs; // Создаем объект для работы с памятью



// ── Настройка динамического роутинга логов ──
enum LogRoute {
    LOG_ONLY_BT  = 1, // Только Блютуз
    LOG_ONLY_USB = 2, // Только USB провод
    LOG_BOTH     = 3  // И туда, и туда
};

LogRoute currentLogMode = LOG_BOTH;

// Тройка макросов под все случаи жизни
#define LOG_PRINTF(...) { \
    if (currentLogMode == LOG_ONLY_USB || currentLogMode == LOG_BOTH) Serial.printf(__VA_ARGS__); \
    if (currentLogMode == LOG_ONLY_BT  || currentLogMode == LOG_BOTH) SerialBT.printf(__VA_ARGS__); \
}

#define LOG_PRINTLN(x) { \
    if (currentLogMode == LOG_ONLY_USB || currentLogMode == LOG_BOTH) Serial.println(x); \
    if (currentLogMode == LOG_ONLY_BT  || currentLogMode == LOG_BOTH) SerialBT.println(x); \
}

#define LOG_PRINT(x) { \
    if (currentLogMode == LOG_ONLY_USB || currentLogMode == LOG_BOTH) Serial.print(x); \
    if (currentLogMode == LOG_ONLY_BT  || currentLogMode == LOG_BOTH) SerialBT.print(x); \
}


// ╔══════════════════════════════════════════════╗
// ║            MAIN SETTINGS                     ║
// ╚══════════════════════════════════════════════╝

// ── I2C ──────────────────────────────
#define SDA_PIN  21
#define SCL_PIN  22

// ── Motor pins ───────────────────────
#define STEP_PIN   33
#define DIR_PIN    32

// ── Buzzer ───────────────────────────
#define BUZZER_PIN 27

// ── Light sensors ────────────────────
#define SENSOR_MOTOR 34    
#define SENSOR_STEPPER  35   

// ── PCA9685 (Servos) ─────────────────
#define PCA_ADDR     0x40
#define PCA_FREQ     50
#define SERVO_COUNT  4
#define PULSE_MIN    150
#define PULSE_MAX    600

// ── OLED ─────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

// ── Violin servo ─────────────────────
#define VIOLIN_SERVO  4

#define VIOLIN_START 25 //начало скрипки 

// ── IBT-2 Violin lift ────────────────
#define RPWM 25        
#define LPWM 26        

// ╔══════════════════════════════════════════════╗
// ║            MG MOTORS (Shoulder/Elbow/Lift)   ║
// ╚══════════════════════════════════════════════╝

// ── Arm link lengths (mm) ────────────
const float L1 = 260.0;
const float L2 = 310.0;

// ── Gear ratio and speed ─────────────
const float GEAR_RATIO_ARM  = 10.0;
const float GEAR_RATIO_LIFT = 36.0;

const float MAX_SPEED_SHOULDER = 450.0; // Подняли скорость для Плеча (ID 2)
const float MAX_SPEED_ELBOW    = 450.0; // Оставили базовую скорость для Локтя (ID 1)

// ── Motor IDs ────────────────────────
const uint8_t SHOULDER_ID = 2;
const uint8_t ELBOW_ID    = 1;
const uint8_t LIFT_ID     = 3; // Новый ID мотора подъема всей руки

struct MotorFeedback {
    int8_t   temperature;
    int16_t  torque;
    int16_t  speed;
    uint16_t encoder;
    bool     valid;
    uint32_t timestamp_us;
};

// Три структуры фидбека вместо двух
MotorFeedback shoulder_fb;
MotorFeedback elbow_fb;
MotorFeedback lift_fb; 
MotorFeedback last_fb;

// Менеджер шины (Round-Robin)
bool motor_busy = false;
uint32_t motor_send_time = 0;
uint8_t rx_buf[13];
int rx_count = 0;
uint8_t current_motor = 0;

// Флаг режима: true — нормальная работа, false — режим только чтения (безопасный тест)
bool automatic_loop_enabled = true;



// ╔══════════════════════════════════════════════════════════════════════════╗
// ║ ВНИМАНИЕ: КООРДИНАТЫ НИЖЕ ПЕРЕЗАПИСЫВАЮТСЯ ИЗ ПАМЯТИ (NVS) ПРИ СТАРТЕ!   ║
// ║ Это лишь базовые (заводские) значения. Актуальные данные хранятся в ESP. ║
// ║ (Для сброса к этим значениям используйте команду /reset_mem)             ║
// ╚══════════════════════════════════════════════════════════════════════════╝


// Массивы теперь строго на 3 элемента
const uint8_t MOTOR_IDS[3] = {SHOULDER_ID, ELBOW_ID, LIFT_ID};
float motor_target_angle[3] = {67.6, 141.9, 90};

float motor_target_speed[3] = {50, 50, 50};

bool motor_torque_enabled[3] = {true, true, true};

// ── Offsets (degrees) ────────────────
const float SHOULDER_OFFSET = 90.0;
const float ELBOW_OFFSET = 90.0;
const float LIFT_OFFSET = 0.0; // Твой офсет для мотора подъема

// ── Trajectory Coordinates (mm) ──────
//float P1_X = 284.5, P1_Y = 303.2; 
float P1_X = 270.5,  P1_Y = 303.9;
float P0_X = 536.1,  P0_Y = -59.8;

//const float P0_X = 587.6, P0_Y = -21.2;  

const float DT = 0.01;                  // 100 Hz update rate

float arm_target_x = P0_X;
float arm_target_y = P0_Y;
bool arm_moving = false;

// ── UART Definitions ─────────────────
#define MotorSerial   Serial1   // UART1 — MG Motors
#define MegaSerial    Serial2   // UART2 — Arduino Mega

// ── UART1 (MG Motors, 3 pcs) ─────────
#define UART1_TX  14
#define UART1_RX  13
#define UART1_BAUD 230400

// ── UART2 (Arduino Mega) ─────────────
#define UART2_TX  17
#define UART2_RX  16
#define UART2_BAUD 115200

// ── ESP-NOW (Secondary ESP32) ────────
uint8_t flipperMAC[] = {0x70, 0x4B, 0xCA, 0x4E, 0x27, 0xD8};
char espnow_cmd_buf[251];
volatile bool has_espnow_cmd = false;
portMUX_TYPE espnow_mux = portMUX_INITIALIZER_UNLOCKED; // Protection for ESP-NOW

// ── Bluetooth ────────────────────────
BluetoothSerial SerialBT;

// ── Note positions (mm) ──────────────
//long x_note[7] = {18, 35, 52, 66, 80, 94, 110};


// ╔══════════════════════════════════════════════════════════════════════════╗
// ║ ВНИМАНИЕ: УГЛЫ ВСЕХ СЕРВОПРИВОДОВ И МОТОРА ЛИФТА ХРАНЯТСЯ В ПАМЯТИ!      ║
// ║ Значения ниже будут проигнорированы, если в памяти есть сохраненные      ║
// ║ откалиброванные данные (команда /save).                                  ║
// ╚══════════════════════════════════════════════════════════════════════════╝


// ── Note positions (mm) ──────────────
// Индексы: [номер струны 0..3][номер ноты 0..6]
// Струна 1, Струна 2, Струна 3, Струна 4
// 1. Базовые заводские координаты в миллиметрах (дробные)
const float default_x_note_mm[4][7] = {
    {18.96, 37.22, 52.22, 67.59, 80.00, 98.89, 111.44}, // Струна 1
    {18.26, 35.96, 49.56, 65.96, 76.15, 95.11, 107.74}, // Струна 2 
    {19.30, 37.04, 50.30, 65.85, 76.85, 95.07, 106.04}, // Струна 3
    {18.15, 35.56, 50.67, 65.26, 75.07, 93.67, 105.48}  // Струна 4
};

// 2. Пустой рабочий массив для хранения шагов (как требует библиотека)
long x_note[4][7];

// ── Physical Fingers Geometry (mm) (Железные координаты 4-х пальцев) ──
// [0]-Указательный, [1]-Средний, [2]-Безымянный, [3]-Мизинец
long physical_fingers[4] = {35, 66, 80, 110};


///Струна 1: 18.00 35.00 52.00 66.00 80.00 94.00 110.00
///<< Струна 2: 16.00 35.00 52.00 66.00 80.00 98.00 110.00
///<< Струна 3: 18.00 33.00 52.00 67.00 78.00 95.00 108.00
///<< Струна 4: 18.00 35.00 52.00 66.00 80.00 94.00 110.00


/*
    _finger_pos[0] = notes[1];
    _finger_pos[1] = notes[3];
    _finger_pos[2] = notes[4];
    _finger_pos[3] = notes[6];
*/

// ── Pulses per mm ────────────────────
int pulses_per_mm = 27; //80 // 53

// ── Servo angles: up / down ──────────
//    [0] Index
//    [1] Middle
//    [2] Ring
//    [3] Pinky
/*
int finger_up[4]   = {36,  45,  5,   3};
int finger_down[4] = {0,   0,   50,  50};
*/
int finger_up[4]   = {42,  36,  52,   48};
int finger_down[4] = {55,   25,   65,  34};

int violin_string[4] = {33, 39, 46, 53};


// [0] - 1 струна, [1] - 2 струна, [2] - 3 струна, [3] - 4 струна
//float lift_string[4] = {75.0, 82.5, 90.0, 98.0}; // ОБЯЗАТЕЛЬНО настрой эти углы под свою механику!
float lift_string_p0[4] = {75.0, 86, 92.5, 102.0}; // Высота в начале смычка (P0)
 
float lift_string_p1[4] = {75.0, 86, 92.5, 99.0}; // Высота в конце смычка (P1)


// Индексы массивов: [0] - 1 струна, [1] - 2 струна, [2] - 3 струна, [3] - 4 струна
int bow_pressure_p0[4]  = {70, 85, 80, 60}; // Давление у колодки (начало смычка)
int bow_pressure_mid[4] = {85,  95,  90,  70};  // Давление в центре (максимальный прижим)
int bow_pressure_p1[4]  = {70, 96, 80, 60}; // Давление у конца смычка


int sn_string = 1;// Лучше задать 1 по умолчанию, чтобы не было индекса -1 при старте

// Скорость (градусы в секунду), с которой рука переходит между струнами
float lift_string_speed = 100.0; 


long CALIBR_DISL = 52; // 259 // 518

// ╔══════════════════════════════════════════════╗
// ║            OBJECTS                           ║
// ╚══════════════════════════════════════════════╝

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Spinner ──────────────────────────
const char spinnerFrames[] = {'|', '/', '-', '\\'};
int spinnerIndex = 0;
unsigned long lastSpinner = 0;

String last_command = "";

// ╔══════════════════════════════════════════════╗
// ║            MG MOTOR PROTOCOL                 ║
// ╚══════════════════════════════════════════════╝

// ── Parse motor response ─────────────
void parse_motor_response(uint8_t* buf, int len, MotorFeedback &fb) {
    // Checksum header (bytes 0-3, sum = byte 4)
    uint8_t h_sum = buf[0] + buf[1] + buf[2] + buf[3];
    if (h_sum != buf[4]) {
        fb.valid = false;
        return;
    }
    
    // Checksum data (bytes 5 to len-2, sum = last byte)
    uint8_t d_sum = 0;
    for (int i = 5; i < len - 1; i++) d_sum += buf[i];
    if (d_sum != buf[len - 1]) {
        fb.valid = false;
        return;
    }
    
    // Parse data (only for 0xA4 response, 13 bytes)
    if (len == 13) {
        fb.temperature = (int8_t)buf[5];
        memcpy(&fb.torque,  &buf[6], 2);
        memcpy(&fb.speed,   &buf[8], 2);
        memcpy(&fb.encoder, &buf[10], 2);
    }
    
    fb.valid = true;
    fb.timestamp_us = micros();
}

// ╔══════════════════════════════════════════════╗
// ║            NON-BLOCKING BUZZER               ║
// ╚══════════════════════════════════════════════╝

struct SoundNote {
    uint16_t freq;      // Частота Гц (0 — пауза/тишина)
    uint16_t duration;  // Длительность в мс
};

#define MAX_SOUND_NOTES 13

static SoundNote sound_queue[MAX_SOUND_NOTES];
static uint8_t sound_count = 0;
static uint8_t sound_index = 0;
static unsigned long sound_timer = 0;
static bool sound_playing = false;

// Запуск проигрывания цепочки нот
void play_sound_sequence(const SoundNote* notes, uint8_t count) {
    if (count == 0 || count > MAX_SOUND_NOTES) return;

    for (uint8_t i = 0; i < count; i++) {
        sound_queue[i] = notes[i];
    }

    sound_count = count;
    sound_index = 0;
    sound_playing = true;
    sound_timer = millis();

    if (sound_queue[0].freq > 0) {
        tone(BUZZER_PIN, sound_queue[0].freq);
    } else {
        noTone(BUZZER_PIN);
    }
}

// Фоновое обновление звука (вызывать в loop)
void buzzer_update() {
    if (!sound_playing) return;

    if (millis() - sound_timer >= sound_queue[sound_index].duration) {
        sound_index++;
        if (sound_index < sound_count) {
            sound_timer = millis();
            if (sound_queue[sound_index].freq > 0) {
                tone(BUZZER_PIN, sound_queue[sound_index].freq);
            } else {
                noTone(BUZZER_PIN);
            }
        } else {
            sound_playing = false;
            noTone(BUZZER_PIN);
        }
    }
}


// ── Готовые пресеты ответов ──

// Утвердительный сигнал "ДА / OK" (двойной короткий пик вверх)
void beep_yes() {
    static const SoundNote seq[] = {
        {1760, 40}, // Нота Ля6
        {0,    20}, // Пауза
        {2637, 60}  // Нота Ми7
    };
    play_sound_sequence(seq, 3);
}

// Отрицательный сигнал "НЕТ / ОШИБКА" (низкий нисходящий тон)
void beep_no() {
    static const SoundNote seq[] = {
        {400,  70}, 
        {0,    20}, 
        {250,  120}
    };
    play_sound_sequence(seq, 3);
}

// ── Выразительные (длинные) звуковые команды ──
void beep_yes_long() {
    static const SoundNote seq[] = {
        {523,  120}, // До (C5)
        {0,    30},  // Пауза
        {659,  120}, // Ми (E5)
        {0,    30},  // Пауза
        {784,  120}, // Соль (G5)
        {0,    30},  // Пауза
        {1047, 400}  // До (C6) — длинный и радостный финал
    };
    play_sound_sequence(seq, 7);
}

void beep_no_long() {
    static const SoundNote seq[] = {
        {587, 150}, // Ре (D5)
        {0,    40}, // Пауза
        {494, 150}, // Си (B4)
        {0,    40}, // Пауза
        {370, 500}  // Фа-диез (F#4) — длинный, низкий, грустный тон
    };
    play_sound_sequence(seq, 5);
}



// ── Error beep ───────────────────────
void beep_error() {
    tone(BUZZER_PIN, 2000, 50);  // 2kHz, 100ms
}

// ── Motor angle limits ───────────────
bool check_motor_limit(uint8_t motor_id, float angle) {
    if (motor_id == SHOULDER_ID && (angle < 50.0 || angle > 103.0)) {
        LOG_PRINTF("[ARM] BLOCKED! Shoulder %.1f out of [67..103]\n", angle);
        beep_error();
        return false;
    }
    if (motor_id == ELBOW_ID && (angle < 90.0 || angle > 185.0)) {
        LOG_PRINTF("[ARM] BLOCKED! Elbow %.1f out of [90..180]\n", angle);
        beep_error();
        return false;
    }
    // НАЧАЛО ВСТАВКИ: Лимиты для мотора подъема
    if (motor_id == LIFT_ID && (angle < 0.0 || angle > 130.0)) { // Задай свои безопасные углы
        LOG_PRINTF("[ARM] BLOCKED! Lift %.1f out of [0..150]\n", angle);
        beep_error();
        return false;
    }
    // КОНЕЦ ВСТАВКИ
    return true;
}

// ── Send A4 command (non-blocking) ───
void send_cmd_a4(uint8_t motor_id, float angle_deg, float speed_dps) {
      if (!check_motor_limit(motor_id, angle_deg)) return; 

    // Развернутый выбор коэффициента редукции
    float current_ratio;
    if (motor_id == LIFT_ID) {
        current_ratio = GEAR_RATIO_LIFT; // 36.0 для лифта
    } else {
        current_ratio = GEAR_RATIO_ARM;  // 10.0 для плеча и локтя
    }
    
    int64_t angle_val = (int64_t)(angle_deg * current_ratio * 100.0);
    uint32_t speed_val = (uint32_t)(fabs(speed_dps) * current_ratio * 100.0);
    
    uint8_t buf[18];
    buf[0] = 0x3E; buf[1] = 0xA4; buf[2] = motor_id; buf[3] = 0x0C;
    buf[4] = (buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF;
    
    memcpy(&buf[5], &angle_val, 8);
    memcpy(&buf[13], &speed_val, 4);
    
    uint8_t data_sum = 0;
    for (int i = 5; i < 17; i++) data_sum += buf[i];
    buf[17] = data_sum;
    
    while (MotorSerial.available()) MotorSerial.read();
    MotorSerial.write(buf, 18);
    
    motor_busy = true;
    rx_count = 0;
    motor_send_time = micros();
}

// ── Send A4 command (blocking) ───────
void send_cmd_a4_blocking(uint8_t motor_id, float angle_deg, float speed_dps) {


    if (!check_motor_limit(motor_id, angle_deg)) return; 

    // Развернутый выбор коэффициента редукции
    float current_ratio;
    if (motor_id == LIFT_ID) {
        current_ratio = GEAR_RATIO_LIFT; // 36.0 для лифта
    } else {
        current_ratio = GEAR_RATIO_ARM;  // 10.0 для плеча и локтя
    }
    
    int64_t angle_val = (int64_t)(angle_deg * current_ratio * 100.0);
    uint32_t speed_val = (uint32_t)(fabs(speed_dps) * current_ratio * 100.0);
    
    uint8_t buf[18];
    buf[0] = 0x3E; buf[1] = 0xA4; buf[2] = motor_id; buf[3] = 0x0C;
    buf[4] = (buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF;
    
    memcpy(&buf[5], &angle_val, 8);
    memcpy(&buf[13], &speed_val, 4);
    
    uint8_t data_sum = 0;
    for (int i = 5; i < 17; i++) data_sum += buf[i];
    buf[17] = data_sum;
    
    while (MotorSerial.available()) MotorSerial.read();
    MotorSerial.write(buf, 18);
    MotorSerial.flush();
    
    uint32_t start = micros();
    int received = 0;
    while (received < 13 && (micros() - start) < 3000) {
        if (MotorSerial.available()) {
            MotorSerial.read();
            received++;
        }
    }
}

// ── Receive motor feedback ───────────
bool motor_receive(MotorFeedback &fb) {
    while (MotorSerial.available() && rx_count < 13) {
        rx_buf[rx_count++] = MotorSerial.read();
    }
    
    if (rx_count >= 13) {
        parse_motor_response(rx_buf, 13, fb);
        motor_busy = false;
        return true;
    }
    
    if (micros() - motor_send_time > 3000) {
        fb.valid = false;
        motor_busy = false;
        return true;
    }
    
    return false;
}

// ╔══════════════════════════════════════════════╗
// ║            ARM KINEMATICS & TRAJECTORY       ║
// ╚══════════════════════════════════════════════╝

// ── Inverse kinematics ───────────────
void calculate_ik(float x, float y, float &m1_deg, float &m2_deg) {
    float d = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    if (d < -1.0f) d = -1.0f;
    if (d > 1.0f) d = 1.0f;
    
    float q2 = acos(d);
    float q1 = atan2(y, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    
    m1_deg = q1 * 180.0f / PI; 
    m2_deg = q2 * 180.0f / PI;
}

// ── Forward kinematics ───────────────
void calculate_fk(float q1_deg, float q2_deg, float &x, float &y) {
    float q1 = q1_deg * PI / 180.0f;
    float q2 = q2_deg * PI / 180.0f;
    x = L1 * cos(q1) + L2 * cos(q1 + q2);
    y = L1 * sin(q1) + L2 * sin(q1 + q2);
}

// ── Set arm target point ─────────────
// ── Set arm target point (С раздельными скоростями) ──
void arm_set_target(float x, float y, float speed_shoulder, float speed_elbow) {
    arm_target_x = x;
    arm_target_y = y;
    arm_moving = true;
    
    motor_target_speed[0] = speed_shoulder; // Скорость для Плеча (индекс 0)
    motor_target_speed[1] = speed_elbow;    // Скорость для Локтя (индекс 1)
}

/*
// ── Независимая установка высоты руки (3 мотор) ──
void arm_set_lift(float angle_deg, float speed_dps) {
    motor_target_angle[2] = angle_deg + LIFT_OFFSET;
    motor_target_speed[2] = speed_dps;
}
    */


unsigned long last_lift_cmd_time = 0; // Память о времени старта
bool lift_measurement_active = false; // Флаг, чтобы напечатать время только 1 раз

// ── Независимая установка высоты руки (3 мотор) ──
// ── Независимая установка высоты руки (3 мотор) ──
void arm_set_lift(float angle_deg, float speed_dps) {
    motor_target_angle[2] = angle_deg + LIFT_OFFSET;
    motor_target_speed[2] = speed_dps;
    
    last_lift_cmd_time = millis(); // Запускаем секундомер
    lift_measurement_active = true; // Говорим программе: "следи за остановкой"
}



// ── Расчет трарории (Уровень приложения) ──


void    arm_trajectory_update() {
    if (!arm_moving) return;
    
    // ОЗК считает геометрию только для 2D плоскости смычка
    float m1_raw, m2_raw;
    calculate_ik(arm_target_x, arm_target_y, m1_raw, m2_raw);
    
    // Пишем уставки в буфер для менеджера шины
    motor_target_angle[0] = m1_raw + SHOULDER_OFFSET;
    motor_target_angle[1] = m2_raw + ELBOW_OFFSET;
    
    arm_moving = false; // Математика отработала, углы обновлены
}
// ── Универсальный менеджер шины (Уровень железа) ──
void motor_bus_update() {
    // Если ждем ответ от мотора
    if (motor_busy) {
        MotorFeedback *fb_ptr;
        if (current_motor == 0)      fb_ptr = &shoulder_fb;
        else if (current_motor == 1) fb_ptr = &elbow_fb;
        else                         fb_ptr = &lift_fb;
        
        if (motor_receive(*fb_ptr)) {
            motor_busy = false; 
            current_motor = (current_motor + 1) % 3; // Переключаемся на следующий мотор (0->1->2->0)
        }
        return; 
    }

    //Если мотор выключен программно, пропускаем его!

    if (!motor_torque_enabled[current_motor]) {
        current_motor = (current_motor + 1) % 3; // Идем к следующему
        return;
    }

    
    // Защита: если целевой угол не проходит программный лимит,
    // send_cmd_a4 сбросит отправку. Чтобы не зависнуть на этом моторе,
    // проверяем лимит заранее и в случае блока просто шагаем дальше.
    if (!check_motor_limit(MOTOR_IDS[current_motor], motor_target_angle[current_motor])) {
        current_motor = (current_motor + 1) % 3;
        return;
    }

    // Шина свободна и лимиты пройдены — шлем текущему мотору актуальную уставку
    send_cmd_a4(MOTOR_IDS[current_motor],
                motor_target_angle[current_motor],
                motor_target_speed[current_motor]);
}


// ── Motor ON/OFF (blocking) ──────────
void set_motor_state_blocking(uint8_t motor_id, bool state) {
    uint8_t cmd_code = state ? 0x88 : 0x81;
    uint8_t cmd[5] = {0x3E, cmd_code, motor_id, 0x00, 0};
    cmd[4] = (cmd[0] + cmd[1] + cmd[2] + cmd[3]) & 0xFF;
    
    while (MotorSerial.available()) MotorSerial.read();
    MotorSerial.write(cmd, 5);
    MotorSerial.flush();
    
    uint32_t start = micros();
    int received = 0;
    while (received < 5 && (micros() - start) < 3000) {
        if (MotorSerial.available()) {
            MotorSerial.read();
            received++;
        }
    }
    LOG_PRINTF("[MG] Motor %d: %s\n", motor_id, state ? "ON" : "OFF");
}

// ── Read motor angle (blocking) ──────
float mg_read_angle(uint8_t motor_id) {
    uint8_t cmd[5] = {0x3E, 0x92, motor_id, 0x00, 0};
    cmd[4] = (cmd[0] + cmd[1] + cmd[2] + cmd[3]) & 0xFF;
    
    while (MotorSerial.available()) MotorSerial.read();
    MotorSerial.write(cmd, 5);
    MotorSerial.flush();
    
    uint8_t resp[14];
    int received = 0;
    uint32_t start = micros();
    
    while (received < 14 && (micros() - start) < 5000) {
        if (MotorSerial.available()) {
            resp[received++] = MotorSerial.read();
        }
    }
    
    if (received < 14) return -999.0;
    
    int64_t angle_raw = 0;
    memcpy(&angle_raw, &resp[5], 8);
    
    // Развернутый выбор редукции для обратного пересчета в градусы
    float current_ratio;
    if (motor_id == LIFT_ID) {
        current_ratio = GEAR_RATIO_LIFT; // 36.0 для лифта
    } else {
        current_ratio = GEAR_RATIO_ARM;  // 10.0 для плеча и локтя
    }
    
    return (float)angle_raw / (current_ratio * 100.0);
}

bool mg_motors_arrived() {
    // Проверяем, что скорость всех трех моторов близка к 0
    // В фидбеке моторов MG скорость обычно отдается в rpm или dps (целое число)
    if (abs(shoulder_fb.speed) > 2) return false;
    if (abs(elbow_fb.speed) > 2)    return false;
    if (abs(lift_fb.speed) > 2)     return false;

    // Дополнительно можно проверять дельту по углам, 
    // но для редукторных моторов MG показатель "скорость = 0" 
    // при активном удержании — это уже 99% гарантия доезда.
    return true;
}

/*
bool mg_lift_arrived() {
    // lift_fb обновляется в motor_bus_update через round-robin
    return (abs(lift_fb.speed) <= 3);
}


    */

#define MG_TICKS_PER_DEG 195.0f

static inline float mg_normalize_encoder(uint16_t enc_raw) {
    return (float)enc_raw / MG_TICKS_PER_DEG;
}

static inline float mg_encoder_to_deg(uint8_t motor_id) {
    if (motor_id == SHOULDER_ID) return mg_normalize_encoder(shoulder_fb.encoder);
    if (motor_id == ELBOW_ID)    return mg_normalize_encoder(elbow_fb.encoder);
    return mg_normalize_encoder(lift_fb.encoder);   // LIFT_ID по умолчанию
}


unsigned int lift_arrival_timeout_ms = 150;

bool mg_lift_arrived() {
    // 1. Слепая зона 50 мс — фидбек не успел
    if (millis() - last_lift_cmd_time < 50) {
        return false;
    }

    // 2. Таймаут — считаем доехал в любом случае
    unsigned long travel = millis() - last_lift_cmd_time;
    if (travel >= lift_arrival_timeout_ms) {
        if (lift_measurement_active) {
            LOG_PRINTF("[LIFT PROFILER] Таймаут: %lu мс\n", travel);
            lift_measurement_active = false;
        }
        return true;
    }

    // 3. Штатная проверка скорости
    bool is_stopped = (abs(lift_fb.speed) <= 13);

    if (is_stopped && lift_measurement_active) {
        LOG_PRINTF("[LIFT PROFILER] Доехал: %lu мс\n", travel);
        lift_measurement_active = false;
    }

    return is_stopped;
}



// ╔══════════════════════════════════════════════╗
// ║            BOW                               ║
// ╚══════════════════════════════════════════════╝

unsigned long bow_timer = 0;
unsigned long bow_last_time = 0; 
unsigned long bow_timeout = 0;
float bow_position = 1.0; // Starting in the middle, keeps its position after stopping

float bow_speed = 0.0;
int bow_direction = 1;
bool bow_active = false;
bool bow_looping = false;

// ── Bow timer ────────────────────────
void reset_bow_timer() { bow_timer = millis(); }

byte is_bow_timer(unsigned long p_t) {
    return (millis() - bow_timer >= p_t) ? 1 : 0;
}

/*
// Сила подхвата в начале: 0.0 = чистый линейный, больше = резче пик в старте
float BOW_ATTACK = 0.5;   // подбери на слух

float ease_attack(float t) {
    // sqrt(t) даёт крутой подъём в начале (пик скорости на старте),
    // линейно смешиваем с обычным t — чем больше BOW_ATTACK, тем резче подхват
    return BOW_ATTACK * sqrtf(t) + (1.0f - BOW_ATTACK) * t;
}
*/
float BOW_ATTACK = 0.6;   // меньше = резче старт. 1.0 = линейный

float ease_attack(float t) {
    return powf(t, BOW_ATTACK);
}
/*
const float BOW_FAST_THRESHOLD = 1.0;   // соответствует скорости 100 в команде
float ease(float t) {
    if (bow_speed > BOW_FAST_THRESHOLD) {     // строго больше 100 → smoothstep
        return t * t * (3.0 - 2.0 * t);
    } else {                                   // 100 и меньше → линейно
        return t;  
    }
}
*/

// ── НАСТРОЙКА ГИБКОЙ S-КРИВОЙ ──
// 1.0 = чистая линейность (без разгона и торможения)
// 2.0 = квадратичное сглаживание
// 3.0 = классический кубический smoothstep
// 4.0+ = затянутый старт/финиш и резкий пролёт в центре
float bow_p_factor = 1.0; // По умолчанию ставим 1.0 (smoothstep)

// Новая кастомная степенная S-кривая
float ease(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    if (bow_p_factor <= 1.0f) return t; // Если 1.0 — идём по линейной прямой

    if (t <= 0.5f) {
        return 0.5f * powf(2.0f * t, bow_p_factor);
    } else {
        return 1.0f - 0.5f * powf(2.0f * (1.0f - t), bow_p_factor);
    }
}

///////////////////////

// Математически точная обратная функция для расчёта фазы при старте с середины
float inv_ease(float p) {
    if (p <= 0.0f) return 0.0f;
    if (p >= 1.0f) return 1.0f;
    if (bow_p_factor <= 1.0f) return p;

    if (p <= 0.5f) {
        return 0.5f * powf(2.0f * p, 1.0f / bow_p_factor);
    } else {
        return 1.0f - 0.5f * powf(2.0f * (1.0f - p), 1.0f / bow_p_factor);
    }
}
/// =======================================================================================================
unsigned long bow_total_timer = 0;
unsigned long bow_total_ms = 0;

/// =======================================================================================================
void reset_bow_total_timer() {
    bow_total_timer = millis();   // current millis value
}

/// =======================================================================================================
byte is_bow_total_timer(uint16_t p_t) {
  if (millis() - bow_total_timer >= p_t) 
  {
   return 1;
  }
  else 
  {
   return 0;
  }
}

/// =======================================================================================================

// ── Start single bow stroke ──────────
void bow_start_cmd(int dir, unsigned long duration_ms, float speed) {
    reset_bow_total_timer();  
    bow_timeout = duration_ms;
    bow_speed = speed;
    bow_active = true;

    // НОВАЯ ЛОГИКА:
    // dir = 2 -> Цикл (Loop)
    // dir = 0 -> Назад
    // dir = 1 -> Вперед
    
    if (dir == 2) {
        bow_looping = true;
        // Стартуем в ту сторону, где больше места
        bow_direction = (bow_position < 0.5f) ? 1 : -1;
    } 
    else if (dir == 0) {
        bow_looping = false;
        bow_direction = -1; // Принудительное движение назад
    } 
    else {
        // dir == 1 (или любое другое значение) — Вперед
        bow_looping = false;
        bow_direction = 1; 
    }

    // вычислить стартовую позицию от текущего прицела руки
    float dx = P1_X - P0_X;
    float dy = P1_Y - P0_Y;
    float p;
    if (fabsf(dx) > fabsf(dy)) {
        p = (arm_target_x - P0_X) / dx;
    } else {
        p = (arm_target_y - P0_Y) / dy;
    }
    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;

    bow_position = inv_ease(p);

    reset_bow_timer();
    bow_last_time = millis();
}

// ── Start looping bow ────────────────
void bow_start_loop(unsigned long duration_ms, float speed) {
    bow_timeout = duration_ms;
    bow_speed = speed;
    bow_active = true;
    bow_looping = true;
    bow_direction = 1;
    bow_position = 0.0;
    
    reset_bow_timer();
    bow_last_time = millis();
}

/*
// ── Bow update ───────────────────────
void bow_update() {
    // Статические переменные для фильтрации и шага
    static float prev_target_a1 = 0.0f;
    static float prev_target_a2 = 0.0f;
    static float filtered_spd1 = 0.0f;
    static float filtered_spd2 = 0.0f;
    static bool reset_prev_angles = true;
    static unsigned long last_bow_tick = 0;

    if (!bow_active) {
        reset_prev_angles = true;
        return;
    }

   // Ограничиваем расчеты строго частотой 100 Гц (раз в 10 мс)
    if (millis() - last_bow_tick < 10) return;
    last_bow_tick = millis();
    
    // Прерываем движение, если истек общий таймаут команды
    if (is_bow_timer(bow_timeout) == 1) {
        bow_active = false;
        bow_total_ms = millis() - bow_total_timer;
        LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms\n", last_command.c_str(), bow_total_ms);
        return;
    }
    
    // Теперь dt всегда железно равен 10 мс (0.01 секунды)
    float dt = 0.01f; 
    
    bow_position += bow_direction * bow_speed * dt;
    
    // Проверка верхнего программного упора
// Проверка верхнего программного упора
    if (bow_position >= 1.0f) { 
        bow_position = 1.0f; 
        if (bow_looping) {
            bow_direction = -1;
        } else {
            bow_active = false;
            bow_total_ms = millis() - bow_total_timer;
            // Добавили печать лога при касании верхнего края!
            LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms (Hit Upper Limit)\n", last_command.c_str(), bow_total_ms);
        }
    }
    // Проверка нижнего программного упора
    else if (bow_position <= 0.0f) { 
        bow_position = 0.0f; 
        if (bow_looping) {
            bow_direction = 1;
        } else {
            bow_active = false;
            bow_total_ms = millis() - bow_total_timer;
            // Добавили печать лога при касании нижнего края!
            LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms (Hit Lower Limit)\n", last_command.c_str(), bow_total_ms);
        }
    }
    
    // Расчет текущих координат по линии ведения сглаживания ease
    float p = ease(bow_position);
    float x = P0_X + (P1_X - P0_X) * p;
    float y = P0_Y + (P1_Y - P0_Y) * p;

    // Математика обратной кинематики
    float m1_raw, m2_raw;
    calculate_ik(x, y, m1_raw, m2_raw);
    float target_a1 = m1_raw + SHOULDER_OFFSET;
    float target_a2 = m2_raw + ELBOW_OFFSET;

    if (reset_prev_angles) {
        prev_target_a1 = target_a1;
        prev_target_a2 = target_a2;
        filtered_spd1 = 0.0f;
        filtered_spd2 = 0.0f;
        reset_prev_angles = false;
    }

    // Считаем сырую производную скорости
    float raw_speed_shoulder = fabsf(target_a1 - prev_target_a1) / dt;
    float raw_speed_elbow    = fabsf(target_a2 - prev_target_a2) / dt;

    // 🔴 КРИТИЧЕСКИЙ ФИКС 2: Экспоненциальный фильтр бегущего среднего (EMA)
    // 0.15 — коэффициент сглаживания. Чем он меньше, тем плавнее скорость.
    float alpha = 0.15f; 
    filtered_spd1 = filtered_spd1 + alpha * (raw_speed_shoulder - filtered_spd1);
    filtered_spd2 = filtered_spd2 + alpha * (raw_speed_elbow - filtered_spd2);

    // Минимальная полка скорости, чтобы мотор не застревал на трении в самом конце
    if (filtered_spd1 < 12.0f && bow_active) filtered_spd1 = 12.0f;
    if (filtered_spd2 < 12.0f && bow_active) filtered_spd2 = 12.0f;

    // Ограничиваем сверху твоими максимальными значениями
    if (filtered_spd1 > MAX_SPEED_SHOULDER) filtered_spd1 = MAX_SPEED_SHOULDER;
    if (filtered_spd2 > MAX_SPEED_ELBOW)    filtered_spd2 = MAX_SPEED_ELBOW;

    // Сохраняем значения для следующего шага через 10 мс
    prev_target_a1 = target_a1;
    prev_target_a2 = target_a2;

    // Отправляем в буфер траектории уже чистую, отфильтрованную скорость
    arm_set_target(x, y, filtered_spd1, filtered_spd2);
}
*/
// ── Стабильный Bow update с фиксацией 100 Гц и прореживанием 3-го мотора ──
void bow_update() {
    static float prev_target_a1 = 0.0f;
    static float prev_target_a2 = 0.0f;
    static float filtered_spd1 = 0.0f;
    static float filtered_spd2 = 0.0f;
    static bool reset_prev_angles = true;
    static unsigned long last_bow_tick = 0;
    
    // НОВОЕ: Память для прореживания команд 3-го мотора
    static float sent_target_a3 = -999.0f;

    if (!bow_active) {
        reset_prev_angles = true;
        sent_target_a3 = -999.0f; // Сбрасываем память лифта при остановке смычка
        return;
    }

    // Ограничиваем расчеты строго частотой 100 Гц (раз в 10 мс)
    if (millis() - last_bow_tick < 10) return;
    last_bow_tick = millis();
    
    if (is_bow_timer(bow_timeout) == 1) {
        bow_active = false;
        bow_total_ms = millis() - bow_total_timer;
        LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms\n", last_command.c_str(), bow_total_ms);
        return;
    }
    
    float dt = 0.01f; 
    
    // --- Динамическое торможение в зоне сингулярности ---
    float temp_p = ease(bow_position);
    float temp_x = P0_X + (P1_X - P0_X) * temp_p;
    float temp_y = P0_Y + (P1_Y - P0_Y) * temp_p;
    float d = (temp_x * temp_x + temp_y * temp_y - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    
    float dampening = 1.0f;
    float d_threshold = 0.82f;
    if (d > d_threshold) {
        dampening = 1.0f - ((d - d_threshold) / (1.0f - d_threshold)) * 0.85f;
        if (dampening < 0.15f) dampening = 0.15f; 
    }

    bow_position += bow_direction * (bow_speed * dampening) * dt;
    

    if (bow_position >= 1.0f) { 
        bow_position = 1.0f; 
        if (bow_looping) {
            bow_direction = -1;
        } else {
            bow_active = false;
            bow_total_ms = millis() - bow_total_timer;
            LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms (Hit Upper Limit)\n", last_command.c_str(), bow_total_ms);
        }
    }
    // Проверка нижнего программного упора
    else if (bow_position <= 0.0f) { 
        bow_position = 0.0f; 
        if (bow_looping) {
            bow_direction = 1;
        } else {
            bow_active = false;
            bow_total_ms = millis() - bow_total_timer;
            LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms (Hit Lower Limit)\n", last_command.c_str(), bow_total_ms);
        }
    }
    
    if (bow_position >= 1.0f) { 
        bow_position = 1.0f; 
        if (bow_looping) bow_direction = -1;
        else { bow_active = false; bow_total_ms = millis() - bow_total_timer; }
    }
    else if (bow_position <= 0.0f) { 
        bow_position = 0.0f; 
        if (bow_looping) bow_direction = 1;
        else { bow_active = false; bow_total_ms = millis() - bow_total_timer; }
    }

    // Получаем реальную пространственную позицию
    float p = ease(bow_position);

    // ==========================================
    // УПРАВЛЕНИЕ 3-М МОТОРОМ (С ПРОРЕЖИВАНИЕМ)
    // ==========================================
    int str_idx = sn_string - 1; 
    if (str_idx >= 0 && str_idx <= 3) {
        // Считаем идеальную математическую высоту
        float ideal_a3 = lift_string_p0[str_idx] + (lift_string_p1[str_idx] - lift_string_p0[str_idx]) * p;
        

                // Отправляем команду мотору ТОЛЬКО если разница накопилась больше 0.25 градуса
        if (fabsf(ideal_a3 - sent_target_a3) > 0.25f || sent_target_a3 == -999.0f) {
            
            // Если разница больше 3 градусов — это смена струны! Едем быстро (lift_string_speed).
            // Если разница маленькая — это микрокоррекция при ведении смычка. Едем плавно (20.0f).
            float move_speed = (fabsf(ideal_a3 - sent_target_a3) > 3.0f) ? lift_string_speed : 20.0f;
            
            sent_target_a3 = ideal_a3;
            arm_set_lift(sent_target_a3, move_speed); 
        }


    }

    // ==========================================
    // УПРАВЛЕНИЕ РУКОЙ (1 и 2 МОТОРЫ - 100 Гц)
    // ==========================================
    float x = P0_X + (P1_X - P0_X) * p;
    float y = P0_Y + (P1_Y - P0_Y) * p;

    float m1_raw, m2_raw;
    calculate_ik(x, y, m1_raw, m2_raw);
    float target_a1 = m1_raw + SHOULDER_OFFSET;
    float target_a2 = m2_raw + ELBOW_OFFSET;

    if (reset_prev_angles) {
        prev_target_a1 = target_a1;
        prev_target_a2 = target_a2;
        filtered_spd1 = 0.0f;
        filtered_spd2 = 0.0f;
        reset_prev_angles = false;
    }

    float raw_speed_shoulder = fabsf(target_a1 - prev_target_a1) / dt;
    float raw_speed_elbow    = fabsf(target_a2 - prev_target_a2) / dt;

    float alpha = 0.13f; 
    filtered_spd1 = filtered_spd1 + alpha * (raw_speed_shoulder - filtered_spd1);
    filtered_spd2 = filtered_spd2 + alpha * (raw_speed_elbow - filtered_spd2);

    if (filtered_spd1 < 12.0f && bow_active) filtered_spd1 = 12.0f;
    if (filtered_spd2 < 12.0f && bow_active) filtered_spd2 = 12.0f;

    if (filtered_spd1 > MAX_SPEED_SHOULDER) filtered_spd1 = MAX_SPEED_SHOULDER;
    if (filtered_spd2 > MAX_SPEED_ELBOW)    filtered_spd2 = MAX_SPEED_ELBOW;

    prev_target_a1 = target_a1;
    prev_target_a2 = target_a2;

    arm_set_target(x, y, filtered_spd1, filtered_spd2);
}

// ╔══════════════════════════════════════════════╗
// ║            VIOLIN LIFT MOTOR                 ║
// ╚══════════════════════════════════════════════╝


// Синхронный подъём/опускание для /up_to_play и /down_to_play
bool        play_lift_active    = false;
unsigned long play_lift_end_ms  = 8000;

int lift_speed = 250;
bool lifting_up = false;
bool lifting_down = false;
unsigned long lift_timer = 0;
unsigned long lift_down_duration = 5000;

bool lift_is_up = false;

void check_initial_lift_position() {
    if (digitalRead(SENSOR_MOTOR) == HIGH) {
        lift_is_up = true;
        LOG_PRINTLN("[LIFT] Startup: Motor UP");
    } else {
        lift_is_up = false;
        LOG_PRINTLN("[LIFT] Startup: Motor DOWN");
    }
}


// ── Lift down timer ──────────────────
void reset_lift_timer() { lift_timer = millis(); }

byte is_lift_timer(uint16_t p_t) {
    return (millis() - lift_timer >= p_t) ? 1 : 0;
}

unsigned long lift_up_timer = 0;
unsigned long lift_up_timeout = 6500;

// ── Lift up timer ────────────────────
void reset_lift_up_timer() { lift_up_timer = millis(); }

byte is_lift_up_timer(uint16_t p_t) {
    return (millis() - lift_up_timer >= p_t) ? 1 : 0;
}

// ── Lift control ─────────────────────
void lift_stop() {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
    lifting_up = false;
    lifting_down = false;
}

void lift_up(int speed) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, speed);
    lifting_up = true;
    lifting_down = false;
    reset_lift_up_timer();
}

void lift_down(int speed, unsigned long duration) {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
    lifting_down = true;
    lifting_up = false;
    lift_down_duration = duration;
    reset_lift_timer();
}

// ── Lift update ──────────────────────
void lift_update() {
    if (lifting_up) {
        // Условие 1: Сработал датчик верха
        if (digitalRead(SENSOR_MOTOR) == HIGH) { 
            lift_stop();
            lift_is_up = true; // Успешно доехали наверх!
            LOG_PRINTLN("[LIFT] UP sensor");
        } 
        // Условие 2: Вылетел таймаут
        else if (is_lift_up_timer(lift_up_timeout) == 1) {
            lift_stop();
            lift_is_up = true; // Тайм-аут сработал, считаем, что уперлись
            LOG_PRINTLN("[LIFT] UP timeout");
        }
    }

    if (lifting_down) {
        if (is_lift_timer(lift_down_duration) == 1) {
            lift_stop();
            lift_is_up = false; // Точно опустились
            LOG_PRINTLN("[LIFT] DOWN");
        }
    }
}


// ── Синхронный подъём скрипки + руки со смычком ─────────────────
void play_go_up(float bow_angle, float mg_speed, unsigned long dur_ms) {
    // В любом случае передаем MG-мотору (руке) нужный угол
    arm_set_lift(bow_angle, mg_speed);

    // Проверяем, поднята ли уже скрипка (по аппаратному датчику или флагу)
    if (lift_is_up || digitalRead(SENSOR_MOTOR) == HIGH) {
        // Скрипка УЖЕ наверху!
        play_lift_active = false; // Сбрасываем флаг ожидания для all_idle()
        lift_is_up = true;        // Синхронизируем состояние на всякий случай
        
        LOG_PRINTF("[PLAY] UP (SKIPPED): Скрипка уже поднята, ожидание %lu мс пропущено.\n", dur_ms);
    } else {
        // Скрипка опущена — запускаем мотор и взводим таймер
        lift_up(lift_speed);
        
        play_lift_active = true;
        play_lift_end_ms = millis() + dur_ms; 

        LOG_PRINTF("[PLAY] UP: bow=%.1f mg=%.0f dps dur=%lu ms\n", bow_angle, mg_speed, dur_ms);
    }
}

// ── Синхронное опускание скрипки + руки со смычком ──────────────
void play_go_down(float bow_angle, float mg_speed, unsigned long dur_ms) {
    arm_set_lift(bow_angle, mg_speed);
    lift_down(lift_speed, lift_down_duration);

    play_lift_active = true;
    play_lift_end_ms = millis() + dur_ms; // Напрямую прибавляем миллисекунды!

    LOG_PRINTF("[PLAY] DOWN: bow=%.1f mg=%.0f dps dur=%lu ms\n",
        bow_angle, mg_speed, dur_ms);
}

// ── Проверка завершения подъёма/опускания ───────────────────────
bool play_lift_arrived() {
    if (!play_lift_active) return true;
    if (millis() >= play_lift_end_ms) {
        play_lift_active = false;
        LOG_PRINTLN("[PLAY] Завершено по таймеру");
        return true;
    }
    return false;
}


// ╔══════════════════════════════════════════════╗
// ║            OLED SCREEN                       ║
// ╚══════════════════════════════════════════════╝

// ── Startup animation ────────────────
void animateStartup() {
    display.clearDisplay();
    display.display();
    delay(100);

    // Print "EMMA" letter by letter
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    String top = "EMMA";
    for (int i = 0; i < (int)top.length(); i++) {
        display.setCursor(4 + i * 14, 0);
        display.print(top[i]);
        display.display();
        delay(120);
    }

    // Draw lines
    display.drawFastHLine(0, 17, SCREEN_WIDTH, SSD1306_WHITE);
    display.display();
    delay(80);

    display.drawFastHLine(0, 22, SCREEN_WIDTH, SSD1306_WHITE);
    display.display();
    delay(80);

    // Print "MAIN CONTROLLER"
    display.setTextSize(1);
    String bot = "MAIN CONTROLLER";
    for (int i = 0; i < (int)bot.length(); i++) {
        display.setCursor(4 + i * 7, 25);
        display.print(bot[i]);
        display.display();
        delay(60);
    }

    delay(150);

    // Blink effect
    for (int i = 0; i < 3; i++) {
        display.invertDisplay(true);
        delay(80);
        display.invertDisplay(false);
        delay(80);
    }
}

// ── Main screen ──────────────────────
void drawMainScreen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // EMMA title on the left
    display.setTextSize(2);
    display.setCursor(4, 0);
    display.print("EMMA");

    // Spinner on the right
    display.setTextSize(1);
    display.setCursor(118, 0);
    display.print(spinnerFrames[spinnerIndex]);

    // Info area
    display.setTextSize(1);
    display.setCursor(4, 20);
    display.print("READY");

    display.display();
}

// ── Show text on screen ──────────────
void showOnScreen(String text) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(4, 0);
    display.print("EMMA");
    display.setTextSize(1);
    display.setCursor(4, 20);
    display.print(text);
    display.display();
}


// ╔══════════════════════════════════════════════╗
// ║            FINGER SERVOS                     ║
// ╚══════════════════════════════════════════════╝

// ── Angle to PWM pulse ───────────────
int angle_to_pulse(int angle) {
    return map(angle, 0, 180, PULSE_MIN, PULSE_MAX);
}

// ── Activate finger ──────────────────
void activate_finger(int index) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        pwm.setPWM(i, 0, angle_to_pulse(finger_up[i]));
    }
    if (index >= 0 && index < SERVO_COUNT) {
        pwm.setPWM(index, 0, angle_to_pulse(finger_down[index]));
    }
}



// ╔══════════════════════════════════════════════╗
// ║            VIOLIN SERVO                      ║
// ╚══════════════════════════════════════════════╝

unsigned long violin_timer = 0;
bool violin_servo_moving = false;
int violin_speed_time = 1;
int violin_servo_step = 1;
int violin_target = VIOLIN_START;
int violin_current = VIOLIN_START;

// ── Violin servo timer ───────────────
void reset_violin_timer() { violin_timer = millis(); }

byte is_violin_timer(uint16_t p_t) {
    return (millis() - violin_timer >= p_t) ? 1 : 0;
}

// ── Set violin angle ─────────────────
void set_violin_angle(int angle) {
    // Protection limits up to 56
    violin_target = constrain(angle, 0, 56); 
}

// ── Is violin servo ready ────────────
bool is_violin_ready() {
    return !violin_servo_moving;
}

// ── Violin servo update ──────────────
void violin_update() {
    if (violin_target != violin_current) {
        
        // ЕСЛИ СКОРОСТЬ 0 ИЛИ МЕНЬШЕ — МГНОВЕННЫЙ ПРЫЖОК
        if (violin_speed_time <= 0) {
            violin_current = violin_target;
            pwm.setPWM(VIOLIN_SERVO, 0, angle_to_pulse(violin_current));
            violin_servo_moving = false;
        } 
        // ИНАЧЕ — ПЛАВНОЕ ДВИЖЕНИЕ ПО ТАЙМЕРУ
        else {
            if (!violin_servo_moving) {
                violin_servo_moving = true;
                reset_violin_timer();
            }

            // Execute everything ONLY when the timer ticks
            if (is_violin_timer(violin_speed_time) == 1) {
                reset_violin_timer();

                if (violin_target > violin_current) {
                    violin_current += violin_servo_step;
                    if (violin_current > violin_target) violin_current = violin_target;
                } 
                else if (violin_target < violin_current) { 
                    violin_current -= violin_servo_step;
                    if (violin_current < violin_target) violin_current = violin_target;
                }

                // Send command to PCA9685 ONLY after the angle has changed
                pwm.setPWM(VIOLIN_SERVO, 0, angle_to_pulse(violin_current));
            }
        }
    } else {
        violin_servo_moving = false;
    }
}


// ── Select violin string ─────────────
void set_violin_string(int stringNum) {
    if (stringNum >= 1 && stringNum <= 4) {
        set_violin_angle(violin_string[stringNum - 1]);
    }
}


// ╔══════════════════════════════════════════════╗
// ║            BOW PRESSURE SERVO                ║
// ╚══════════════════════════════════════════════╝

// ── Bow Pressure Servo (Прижим смычка) ──
#define BOW_PRESSURE_SERVO  5     // Канал на плате PCA9685 (5-й порт)

#define BOW_PRESSURE_START  20   // Начальный угол (свободное состояние) //160


// ── Настройки динамического давления (в градусах от 40 до 162) ──
bool dynamic_pressure_enabled = false;



// ── Переменные состояния сервы ──
unsigned long bow_pressure_timer = 0;
bool bow_pressure_moving = false;
int bow_pressure_speed_time = 0; // 0 = моментально
int bow_pressure_target = BOW_PRESSURE_START;
int bow_pressure_current = BOW_PRESSURE_START;

// ── Таймер ──
void reset_bow_pressure_timer() { bow_pressure_timer = millis(); }

byte is_bow_pressure_timer(uint16_t p_t) {
    return (millis() - bow_pressure_timer >= p_t) ? 1 : 0;
}

// ── 1. Функция расчета идеального давления ──
void dynamic_pressure_update() {
    if (!dynamic_pressure_enabled) return;

    // Определяем текущую струну (индекс от 0 до 3)
    int str_idx = sn_string - 1; 
    if (str_idx < 0) str_idx = 0;
    if (str_idx > 3) str_idx = 3;

    // Берем позицию смычка (от 0.0 до 1.0) с учетом S-кривой разгона
    float p = ease(bow_position);
    float ideal_pressure = 0;

    // Считаем левую половину смычка (от P0 до Центра)
    if (p <= 0.5f) {
        float factor = (0.5f - p) * 2.0f;     // 1.0 у колодки (P0), 0.0 в центре
        factor = factor * factor;             // Парабола для плавности
        ideal_pressure = bow_pressure_mid[str_idx] + (bow_pressure_p0[str_idx] - bow_pressure_mid[str_idx]) * factor;
    } 
    // Считаем правую половину смычка (от Центра до P1)
    else {
        float factor = (p - 0.5f) * 2.0f;     // 0.0 в центре, 1.0 у конца (P1)
        factor = factor * factor;             // Парабола для плавности
        ideal_pressure = bow_pressure_mid[str_idx] + (bow_pressure_p1[str_idx] - bow_pressure_mid[str_idx]) * factor;
    }

    // Жесткие лимиты механики
    ideal_pressure = constrain(ideal_pressure, 40, 162);

    // Передаем цель мотору
    bow_pressure_target = (int)ideal_pressure;
    bow_pressure_speed_time = 0; // В динамическом режиме серва должна успевать за рукой без задержек
}

// ── 2. Функция физического шагания сервы ──
void bow_pressure_update() {
    if (bow_pressure_target != bow_pressure_current) {
        
        // Резкое движение (или следование за динамическим режимом)
        if (bow_pressure_speed_time <= 0) {
            bow_pressure_current = bow_pressure_target;
            pwm.setPWM(BOW_PRESSURE_SERVO, 0, angle_to_pulse(bow_pressure_current));
            bow_pressure_moving = false;
        } 
        // Плавное ручное движение по таймеру
        else {
            if (!bow_pressure_moving) {
                bow_pressure_moving = true;
                reset_bow_pressure_timer();
            }

            if (is_bow_pressure_timer(bow_pressure_speed_time) == 1) {
                reset_bow_pressure_timer();

                if (bow_pressure_target > bow_pressure_current) bow_pressure_current++;
                else if (bow_pressure_target < bow_pressure_current) bow_pressure_current--;

                pwm.setPWM(BOW_PRESSURE_SERVO, 0, angle_to_pulse(bow_pressure_current));
            }
        }
    } else {
        bow_pressure_moving = false;
    }
}



// ╔══════════════════════════════════════════════╗
// ║            STRING + NOTE (/sn)               ║
// ╚══════════════════════════════════════════════╝

int  sn_step = 0;          // 0=простой 1=поднять 2=таймер+ехать 3=ждать доезд 4=таймер+нажать
int  sn_note = 0;
bool sn_press = true;
unsigned long sn_timer = 0;
unsigned long sn_press_timer = 0;
const uint16_t SN_LIFT_DELAY  = 35;  // мс на физический подъём пальца 

const uint16_t SN_PRESS_DELAY = 0;  // мс паузы перед нажатием пальца  60

unsigned long sn_start_ms = 0;   // момент старта команды /sn
unsigned long sn_total_ms = 0;   // длительность последней команды /sn


bool sn_string_changed = false;   // поменялась ли струна в этой команде

/// =======================================================================================================
void reset_sn_timer() {
    sn_timer = millis();   // current millis value
}

/// =======================================================================================================
byte is_sn_timer(uint16_t p_t) {
  if (millis() - sn_timer >= p_t) 
  {
   return 1;
  }
  else 
  {
   return 0;
  }
}

/// =======================================================================================================
void reset_sn_press_timer() {
    sn_press_timer = millis();   // current millis value
}

/// =======================================================================================================
byte is_sn_press_timer(uint16_t p_t) {
  if (millis() - sn_press_timer >= p_t) 
  {
   return 1;
  }
  else 
  {
   return 0;
  }
}

/// =======================================================================================================

// ── Start /sn sequence ───────────────
void set_string_and_note(int stringNum, int nota, int press) {
    if (sn_step != 0) {
        return;
    }

    sn_string = stringNum;
    sn_note   = nota;
    sn_press  = (press != 0);
    sn_start_ms = millis();      
    sn_step = 1;
}

// ── /sn state machine (Pipelined & Overlapped Edition) ────────────────
void sn_update() {
    switch (sn_step) {

        case 0:
            break;

        case 1:
            // шаг 1: поднять пальцы, запустить таймер (один раз)
            activate_finger(-1);
            reset_sn_timer();
            sn_step = 2;
            break;

        case 2:
            // шаг 2: ждём подъём серв, затем запускаем движение
            if (is_sn_timer(SN_LIFT_DELAY) == 1) {
                int violin_before = violin_current;         
                set_violin_string(sn_string);                 
                sn_string_changed = (violin_target != violin_before);  

                if (sn_string >= 1 && sn_string <= 4) {
                    // --- НОВАЯ УМНАЯ СМЕНА СТРУНЫ ---
                    int str_idx = sn_string - 1;
                    
                    // Строго линейный расчет высоты для НОВОЙ струны
                    // в ТЕКУЩЕЙ точке смычка (bow_position)
                    float target_lift_angle = lift_string_p0[str_idx] + (lift_string_p1[str_idx] - lift_string_p0[str_idx]) * bow_position;
                    
                    arm_set_lift(target_lift_angle, lift_string_speed);
                }

                if (sn_note != 0) {
                    int str_idx = sn_string - 1;       // Индекс струны (0..3)
                    if (str_idx < 0) str_idx = 0;      // Защита от ошибок
                    if (str_idx > 3) str_idx = 3;
                    set_note(sn_note, x_note[str_idx]); // Передаем массив нужной струны
                }
                sn_step = 3;
            }
            break;

            case 3:
            // шаг 3: Контроль полета и УМНОЕ ОПЕРЕЖЕНИЕ сервой
            if (is_violin_ready()) {
                
                // ЕСЛИ СТРУНА НЕ МЕНЯЛАСЬ — включаем умное наложение
                if (!sn_string_changed) {
                    uint32_t steps_left = stepper_get_steps_left();
                    uint32_t total_steps = stepper_get_total_steps();
                    
                    bool lookahead_trigger = false;

                    lookahead_trigger = (steps_left <= 130);


                    if (lookahead_trigger || stepper_is_ready()) {
                        if (sn_note != 0 && sn_press) {
                            activate_finger(get_active_finger());
                        }
                        sn_step = 4;
                    }
                } 
                // ЕСЛИ СТРУНА МЕНЯЛАСЬ — идем по безопасному классическому пути
                else {
                    if (!fingers_are_busy()) {
                        reset_sn_press_timer();    
                        sn_step = 4;
                    }
                }
            }
            break;

        case 4:
            if (!sn_string_changed) {
                if (!fingers_are_busy()) {
                    sn_total_ms = millis() - sn_start_ms;
                    LOG_PRINTF("[SN] Pipelined Jump Total: %lu ms\n", sn_total_ms);
                    sn_step = 0;
                }
            } else {
                // Смена струны: ждём И стабильность деки, И фактического приезда лифта
                if (is_sn_press_timer(SN_PRESS_DELAY) == 1 && mg_lift_arrived()) {
                    if (sn_note != 0 && sn_press) {
                        activate_finger(get_active_finger());
                    }
                    sn_total_ms = millis() - sn_start_ms;
                    LOG_PRINTF("[SN] String Change Total: %lu ms\n", sn_total_ms);
                    sn_step = 0;
                }
            }
            break;
    }
}



// ╔══════════════════════════════════════════════╗
// ║            CALIBRATION                       ║
// ╚══════════════════════════════════════════════╝
/*
// ── Blocking calibration ─────────────
void calibrate() {
    // stepper_move parameters: (steps, direction, speed_max, speed_min, accel, decel)
    stepper_move(400, true, 4000, 1000, 50, 50); 
    while (!stepper_is_ready()) delay(1);

    stepper_move(999999, false, 4000, 1000, 50, 50);

    while (digitalRead(SENSOR_STEPPER) == HIGH) delay(1);

    stepper_stop();

    stepper_move(CALIBR_DISL, true, 4000, 1000, 50, 50);
    while (!stepper_is_ready()) delay(1);

    // Sync the internal absolute coordinates of all fingers!
    fingers_set_home(x_note);
}
    */

    void calibrate() {
    // 1. Плавный отъезд от датчика (всего 200 шагов, разгон и торможение по 80)
    //stepper_move(200, true, 2400, 800, 80, 80); 
    stepper_move(333, true, 2400, 800, 80, 80); 
    while (!stepper_is_ready()) delay(1);

    // 2. Медленный и точный поиск датчика (полка 2400 шаг/сек ~ 90 об/мин)
    stepper_move(999999, false, 2400, 800, 150, 150);
    while (digitalRead(SENSOR_STEPPER) == HIGH) delay(1);
    stepper_stop();

    // 3. Плавный возврат на домашнюю точку
    stepper_move(CALIBR_DISL, true, 2400, 800, 100, 100);
    while (!stepper_is_ready()) delay(1);

    fingers_set_home(physical_fingers);
}

// ── Non-blocking calibration ─────────
int cal_state = 0;
bool calibrating = false;

void calibrate_start() {
    cal_state = 1;
    calibrating = true;
    stepper_stop();
    activate_finger(-1);
    // Ставим мягкие параметры
    //stepper_move(200, true, 2400, 800, 80, 80);
    stepper_move(333, true, 2400, 800, 80, 80);
}

void calibrate_update() {
    if (!calibrating) return;

    switch (cal_state) {
        case 1:
            if (stepper_is_ready()) {
                cal_state = 2;
                // Ищем датчик плавно
                stepper_move(999999, false, 2400, 800, 150, 150);
            }
            break;

        case 2:
            if (digitalRead(SENSOR_STEPPER) == LOW) {
                stepper_stop();
                cal_state = 3;
                // Возвращаемся домой
                stepper_move(CALIBR_DISL, true, 2400, 800, 100, 100);
            }
            break;

        case 3:
            if (stepper_is_ready()) {
                fingers_set_home(physical_fingers);
                cal_state = 0;
                calibrating = false;
                LOG_PRINTLN("calibration done!");
            }
            break;
    }
}

// ╔══════════════════════════════════════════════╗
// ║            COMMAND PARSER                     ║
// ╚══════════════════════════════════════════════╝

String _args[7];
int _argCount = 0;

// ── Split input by ';' ───────────────
void parse(String input) {
    input.trim();
    _argCount = 0;
    
    // 🔴 FIX: Clear the old phantom arguments before parsing!
    for(int i = 0; i < 7; i++) {
        _args[i] = "";
    }
    
    int start = 0;
    while (start < (int)input.length() && _argCount < 7) {
        int sep = input.indexOf(';', start);
        if (sep == -1) sep = input.length();
        _args[_argCount++] = input.substring(start, sep);
        start = sep + 1;
    }
}

// ── Argument accessors ───────────────
String cmd()       { return _args[0]; }
int    getInt(int i) { return _args[i].toInt(); }
float  getFloat(int i) { return _args[i].toFloat(); } // Added for precise IK coordinates




// ╔══════════════════════════════════════════════╗
// ║            SCENARIO QUEUE (/load_test)       ║
// ╚══════════════════════════════════════════════╝

#define QUEUE_MAX  1000          // максимум команд в очереди

char  queue_cmd[QUEUE_MAX][40]; // тексты команд
int   queue_count = 0;          // сколько команд загружено
int   queue_index = 0;          // какая команда сейчас играется
bool  loading  = false;         // идёт приём строк
bool  playing  = false;         // идёт воспроизведение

// ── Войти в режим приёма ─────────────
void queue_load_start() {
    loading = true;
    playing = false;
    queue_count = 0;
    queue_index = 0;
}

// ── Добавить строку в очередь ────────
void queue_add(String line) {
    line.trim();
    if (line.length() == 0) return;            // пустую строку пропустить
    if (queue_count >= QUEUE_MAX) return;      // очередь переполнена

    line.toCharArray(queue_cmd[queue_count], 40);
    queue_count++;
}

// ── Конец приёма ─────────────────────
void queue_load_end() {
    loading = false;
    LOG_PRINTF("[QUEUE] Loaded %d commands\n", queue_count);
}

// ── Запустить воспроизведение ────────
void queue_play() {
    if (queue_count == 0) return;
    playing = true;
    queue_index = 0;
}

// ── Прервать воспроизведение ─────────
void queue_stop() {
    playing = false;
    activate_finger(-1);
    LOG_PRINTLN("[QUEUE] Stopped.");
}

// ── Всё ли свободно ──────────────────
/*
bool all_idle() {
    return (sn_step == 0) && !bow_active && !fingers_are_busy();
}
*/




// ╔══════════════════════════════════════════════╗
// ║            PAUSE (/p)                        ║
// ╚══════════════════════════════════════════════╝

unsigned long pause_timer = 0;
unsigned long pause_duration = 0;
bool pause_active = false;

/// =======================================================================================================
void reset_pause_timer() {
    pause_timer = millis();
}

/// =======================================================================================================
byte is_pause_timer(unsigned long p_t) {
  if (millis() - pause_timer >= p_t) 
  {
   return 1;
  }
  else 
  {
   return 0;
  }
}

/// =======================================================================================================
void pause_start(unsigned long ms) {
    pause_duration = ms;
    reset_pause_timer();
    pause_active = true;
}

/// =======================================================================================================
void pause_update() {
    if (!pause_active) return;
    if (is_pause_timer(pause_duration) == 1) {
        pause_active = false;
    }
}



// ╔══════════════════════════════════════════════╗
// ║            PARALLEL PAUSE + SN (/psn)        ║
// ╚══════════════════════════════════════════════╝

unsigned long psn_timer    = 0;
unsigned long psn_duration = 0;
bool          psn_active   = false;

/// =======================================================================================================
void reset_psn_timer() {
    psn_timer = millis();
}

/// =======================================================================================================
byte is_psn_timer(unsigned long p_t) {
  if (millis() - psn_timer >= p_t) 
  {
   return 1;
  }
  else 
  {
   return 0;
  }
}

/// =======================================================================================================
void psn_start(unsigned long ms, int stringNum, int nota) {
    psn_duration = ms;
    reset_psn_timer();
    psn_active = true;

    // Запускаем перестройку параллельно
    set_string_and_note(stringNum, nota, 1);
}

/// =======================================================================================================
void psn_update() {
    if (!psn_active) return;
    if (is_psn_timer(psn_duration) == 1) {
        psn_active = false;
    }
}



/*
bool all_idle() {
  //  return (sn_step == 0) && !bow_active && !fingers_are_busy() && mg_motors_arrived();
   // return (sn_step == 0) && !bow_active && !fingers_are_busy();
    //return (sn_step == 0) && !bow_active && !fingers_are_busy() && !pause_active;

     return (sn_step == 0) && !bow_active && !fingers_are_busy() && !pause_active && !psn_active;
}

*/
bool all_idle() {
    return (sn_step == 0) && 
           !bow_active && 
           !fingers_are_busy() && 
           !pause_active && 
           !psn_active && 
           play_lift_arrived() && 
           !lifting_up && 
           !lifting_down;
}


// ╔══════════════════════════════════════════════╗
// ║            ARDUINO MEGA                      ║
// ╚══════════════════════════════════════════════╝

void send_to_mega(String cmd) {
    MegaSerial.println(cmd);
    LOG_PRINTLN("[MEGA] -> " + cmd);
}



// ╔══════════════════════════════════════════════╗
// ║            COMMAND EXECUTOR                  ║
// ╚══════════════════════════════════════════════╝

void executeCommand(String input) {
    parse(input);
    last_command = input;  
    showOnScreen(input);

    // ── 1. МЕНЮ ПОМОЩИ ──
    if (cmd() == "/help") {
        LOG_PRINTLN("╔════════════════════════════════════════════════════════╗");
        LOG_PRINTLN("║                     EMMA COMMANDS                      ║");
        LOG_PRINTLN("╚════════════════════════════════════════════════════════╝");
        
        LOG_PRINTLN("--- 💾 ПАМЯТЬ И СИСТЕМА ---");
        LOG_PRINTLN("/save                   - Сохранить все текущие настройки в NVS (Flash)");
        LOG_PRINTLN("/reset_mem              - Очистить память NVS (сброс до заводских)");
        LOG_PRINTLN("/log_mode;X             - Вывод логов: 1=BT, 2=USB, 3=BOTH");
        LOG_PRINTLN("/mega;cmd               - Отправить команду на Arduino Mega");

        LOG_PRINTLN("\n--- 🦾 РУКА И MG МОТОРЫ ---");
        LOG_PRINTLN("/arm_on /arm_off        - Вкл/Выкл удержание для Плеча и Локтя");
        LOG_PRINTLN("/mg_on /mg_off          - Вкл/Выкл удержание для ВСЕХ 3-х MG моторов");
        LOG_PRINTLN("/arm;X;Y;[spd]          - Поехать рукой в точки X, Y (блоками)");
        LOG_PRINTLN("/mg_read                - Считать текущие углы моторов и расчитать XY");
        LOG_PRINTLN("/mg_status              - Вывести телеметрию (темп, момент, скорость, ошибки)");
        LOG_PRINTLN("/mg_enc                 - Вывести сырые и градусные значения энкодеров");
        LOG_PRINTLN("/mg_test;A;S;ID         - Отправить угол(A) и скорость(S) мотору(ID)");

        LOG_PRINTLN("\n--- 🎻 СМЫЧОК И ТРАЕКТОРИЯ ---");
        LOG_PRINTLN("/bow;D;T;S              - Движение: D(1=вперед, 0/-1=назад, 2=цикл), T(мс), S(скорость %)");
        LOG_PRINTLN("/bow_loop;T;S           - Зациклить смычок: T(мс), S(скорость %)");
        LOG_PRINTLN("/bow_stop               - Остановить смычок");
        LOG_PRINTLN("/set_p0;X;Y             - Задать точку колодки (P0)");
        LOG_PRINTLN("/set_p1;X;Y             - Задать точку конца смычка (P1)");
        LOG_PRINTLN("/get_p                  - Вывести текущие координаты P0 и P1");
        LOG_PRINTLN("/bow_p;X                - Задать фактор S-кривой (>= 1.0)");

        LOG_PRINTLN("\n--- ⬆️ ВЫСОТА И ПРИЖИМ (ЛИФТ И СЕРВА) ---");
        LOG_PRINTLN("/set_lift;A             - Прямая установка угла(A) мотора лифта");
        LOG_PRINTLN("/lift_mg_set;A;S        - Установить угол(A) и скорость(S) мотора лифта");
        LOG_PRINTLN("/lift_speed;[S]         - Задать/показать скорость смены струны для лифта");
        LOG_PRINTLN("/set_lift_str;S;P0;P1   - Настроить высоту струны S(1-4) для P0 и P1");
        LOG_PRINTLN("/get_lift_str           - Показать настройки высоты всех струн");
        LOG_PRINTLN("/lift_to;ms             - Установить таймаут доезда лифта (мс)");
        LOG_PRINTLN("/dyn_press;X            - Динамический прижим: 1=ВКЛ, 0=ВЫКЛ");
        LOG_PRINTLN("/set_dyn_press;S;P0;M;P1- Настроить давление для струны S(1-4)");
        LOG_PRINTLN("/get_dyn_press          - Показать настройки прижима");
        LOG_PRINTLN("/bow_press;A;[T]        - Ручная установка давления: Угол(A), Время(T)");

        LOG_PRINTLN("\n--- 🎼 ПАЛЬЦЫ, НОТЫ И СКРИПКА ---");
        LOG_PRINTLN("/sn;S;N;[P]             - Сыграть: Струна(S), Нота(N), Нажать(P: 1/0)");
        LOG_PRINTLN("/psn;T;S;N              - Сыграть /sn параллельно с паузой T(мс)");
        LOG_PRINTLN("/finger;N               - Нажать палец (1-4, -1=поднять все)");
        LOG_PRINTLN("/set_fingers;N;U;D      - Настроить углы сервы пальца N(1-4): Up(U), Down(D)");
        LOG_PRINTLN("/get_fingers            - Показать углы всех пальцев");
        LOG_PRINTLN("/set_note;N             - Позиционировать ноту N(1-7) на текущей струне");
        LOG_PRINTLN("/set_mm;S;N;V           - Задать позицию ноты N на струне S в мм(V)");
        LOG_PRINTLN("/get_mm                 - Показать карту нот (мм)");
        LOG_PRINTLN("/violin;A;[S]           - Повернуть скрипку на угол(A), скорость шага(S)");
        LOG_PRINTLN("/set_v_str;S;A          - Задать угол сервы скрипки для струны S(1-4)");

        LOG_PRINTLN("\n--- ⚙️ IBT-2, ПРИВОД И СЕНСОРЫ ---");
        LOG_PRINTLN("/lift_up;[S]            - Поднять скрипку (IBT-2) [скорость S]");
        LOG_PRINTLN("/lift_down;[S];[T]      - Опустить скрипку (IBT-2) [скорость S, время T]");
        LOG_PRINTLN("/lift_stop              - Стоп подъёма (IBT-2)");
        LOG_PRINTLN("/up_to_play;[A];[S];[T] - Синхронный подъем скрипки в рабочую позицию");
        LOG_PRINTLN("/down_to_play;[A];[S];[T]- Синхронное опускание скрипки");
        LOG_PRINTLN("/sensor_status          - Проверить концевик и статус подъема");

        LOG_PRINTLN("\n--- ⏱️ КАЛИБРОВКА И ПЛЕЕР ---");
        LOG_PRINTLN("/cal                    - Плавная калибровка каретки (асинхронная)");
        LOG_PRINTLN("/calibrate              - Калибровка каретки (блокирующая)");
        LOG_PRINTLN("/p;T                    - Пауза (ожидание) T(мс)");
        LOG_PRINTLN("/load_test              - Начать запись сценария в очередь");
        LOG_PRINTLN("/end                    - Завершить запись сценария");
        LOG_PRINTLN("/play                   - Воспроизвести сценарий");
        LOG_PRINTLN("/stop                   - Остановить воспроизведение");
        LOG_PRINTLN("═════════════════════════════════════════════════════════");


        LOG_PRINTLN("\n--- 🔊 ЗВУК И СИГНАЛЫ ---");
        LOG_PRINTLN("/yes                    - Проиграть яркий звук подтверждения (OK)");
        LOG_PRINTLN("/no                     - Проиграть угасающий звук отказа (NO)");
    }

    // ── 2. СИСТЕМА И ПАМЯТЬ ──
    if (cmd() == "/save") {
        prefs.putBytes("f_up", finger_up, sizeof(finger_up));
        prefs.putBytes("f_down", finger_down, sizeof(finger_down));
        prefs.putBytes("x_note", x_note, sizeof(x_note));
        prefs.putBytes("phys_fing", physical_fingers, sizeof(physical_fingers));
        prefs.putBytes("v_str", violin_string, sizeof(violin_string));
        prefs.putBytes("lift_p0", lift_string_p0, sizeof(lift_string_p0));
        prefs.putBytes("lift_p1", lift_string_p1, sizeof(lift_string_p1));
        prefs.putBytes("bp_p0", bow_pressure_p0, sizeof(bow_pressure_p0));
        prefs.putBytes("bp_mid", bow_pressure_mid, sizeof(bow_pressure_mid));
        prefs.putBytes("bp_p1", bow_pressure_p1, sizeof(bow_pressure_p1));

        prefs.putFloat("P0_X", P0_X);
        prefs.putFloat("P0_Y", P0_Y);
        prefs.putFloat("P1_X", P1_X);
        prefs.putFloat("P1_Y", P1_Y);
        
        LOG_PRINTLN("[SYS] УСПЕХ! Все параметры записаны в энергонезависимую память.");
        
        LOG_PRINTLN("\n=== СОХРАНЕННЫЕ ДАННЫЕ ===");
        LOG_PRINTF("Точки смычка: P0(%.1f, %.1f) | P1(%.1f, %.1f)\n", P0_X, P0_Y, P1_X, P1_Y);
        LOG_PRINTLN("Углы серв пальцев (UP -> DOWN):");
        for (int i = 0; i < 4; i++) {
            LOG_PRINTF("  Палец %d: %d -> %d\n", i + 1, finger_up[i], finger_down[i]);
        }
        LOG_PRINTLN("Физические позиции пальцев (мм):");
        LOG_PRINTF("  [%.2f, %.2f, %.2f, %.2f]\n", 
            (float)physical_fingers[0] / pulses_per_mm, 
            (float)physical_fingers[1] / pulses_per_mm, 
            (float)physical_fingers[2] / pulses_per_mm, 
            (float)physical_fingers[3] / pulses_per_mm);
            
        LOG_PRINTLN("Карта нот (мм):");
        for (int s = 0; s < 4; s++) {
            LOG_PRINTF("  Струна %d: ", s + 1);
            for (int n = 0; n < 7; n++) {
                LOG_PRINTF("%.2f ", (float)x_note[s][n] / pulses_per_mm);
            }
            LOG_PRINTLN("");
        }

        LOG_PRINTLN("Углы сервы скрипки (Струны 1-4):");
        LOG_PRINTF("  [%d, %d, %d, %d]\n", violin_string[0], violin_string[1], violin_string[2], violin_string[3]);

        LOG_PRINTLN("Высота струн Лифта (P0 -> P1):");
        for (int i = 0; i < 4; i++) {
            LOG_PRINTF("  Струна %d: %.1f -> %.1f\n", i + 1, lift_string_p0[i], lift_string_p1[i]);
        }

        LOG_PRINTLN("Динамический прижим (P0 -> MID -> P1):");
        for (int i = 0; i < 4; i++) {
            LOG_PRINTF("  Струна %d: %d -> %d -> %d\n", i + 1, bow_pressure_p0[i], bow_pressure_mid[i], bow_pressure_p1[i]);
        }
        LOG_PRINTLN("==========================\n");

        beep_yes();
    }

    if (cmd() == "/reset_mem") {
        prefs.clear();
        LOG_PRINTLN("[SYS] ПАМЯТЬ ОЧИЩЕНА. Перезагрузите ESP32 для возврата к настройкам по умолчанию.");
    }

    if (cmd() == "/log_mode") {
        int newMode = getInt(1);
        if (newMode >= 1 && newMode <= 3) {
            currentLogMode = (LogRoute)newMode;
            LOG_PRINTF("[SYS] Log mode changed to: %d\n", newMode);
        } else {
            LOG_PRINTF("[SYS] ERROR! Unknown mode: %d. Use 1 (BT), 2 (USB), 3 (BOTH)\n", newMode);
        }
    }

    if (cmd() == "/mega") {
        int firstSep = input.indexOf(';');
        if (firstSep != -1) {
            String megaCmd = input.substring(firstSep + 1); 
            send_to_mega(megaCmd);
        }
    }

    // ── 3. MG МОТОРЫ И РУКА ──
    if (cmd() == "/arm_on") {
        set_motor_state_blocking(MOTOR_IDS[0], true);
        motor_torque_enabled[0] = true;
        delay(5);
        set_motor_state_blocking(MOTOR_IDS[1], true);
        motor_torque_enabled[1] = true;
        arm_set_target(P0_X, P0_Y, 30.0f, 30.0f);
        LOG_PRINTF("[MG] Arm ON. Moving to P0 (%.1f, %.1f) at low speed.\n", P0_X, P0_Y);
    }

    if (cmd() == "/arm_off") {
        set_motor_state_blocking(MOTOR_IDS[0], false);
        motor_torque_enabled[0] = false;
        delay(5);
        set_motor_state_blocking(MOTOR_IDS[1], false);
        motor_torque_enabled[1] = false;
        LOG_PRINTF("[MG] Arm motors (Shoulder & Elbow) OFF. Lift is holding.\n");
    }

    if (cmd() == "/mg_on") {
        for (int i = 0; i < 3; i++) {
            set_motor_state_blocking(MOTOR_IDS[i], true);
            motor_torque_enabled[i] = true;
            delay(5);
        }
    }

    if (cmd() == "/mg_off") {
        for (int i = 0; i < 3; i++) {
            set_motor_state_blocking(MOTOR_IDS[i], false);
            motor_torque_enabled[i] = false;
            delay(5);
        }
    }

    if (cmd() == "/arm") {
        float x = getFloat(1);
        float y = getFloat(2);
        float spd = 50.0;
        if (_argCount >= 4) spd = getFloat(3);
        
        float m1_raw, m2_raw;
        calculate_ik(x, y, m1_raw, m2_raw);
        
        send_cmd_a4_blocking(SHOULDER_ID, m1_raw + SHOULDER_OFFSET, spd);
        send_cmd_a4_blocking(ELBOW_ID, m2_raw + ELBOW_OFFSET, spd);
        
        LOG_PRINTF("[ARM] X=%.1f Y=%.1f S=%.1f E=%.1f\n", 
            x, y, m1_raw + SHOULDER_OFFSET, m2_raw + ELBOW_OFFSET);
    }

    if (cmd() == "/mg_read") {
        bow_active = false;
        arm_moving = false;
        motor_busy = false;
        delay(10);
        while (MotorSerial.available()) MotorSerial.read();
        
        float a1 = mg_read_angle(SHOULDER_ID);
        float a2 = mg_read_angle(ELBOW_ID);
        float a3 = mg_read_angle(LIFT_ID);
        
        float q1 = a1 - SHOULDER_OFFSET;
        float q2 = a2 - ELBOW_OFFSET;
        
        float x, y;
        calculate_fk(q1, q2, x, y);
        LOG_PRINTF("[MG] Shoulder: %.2f  Elbow: %.2f  Lift: %.2f  X: %.1f  Y: %.1f\n", a1, a2, a3, x, y);
    }

    if (cmd() == "/mg_test") {
        float angle = getFloat(1);
        float speed = getFloat(2);
        uint8_t id = getInt(3);
        send_cmd_a4_blocking(id, angle, speed);
        LOG_PRINTF("[MG] Sent: id=%d angle=%.1f speed=%.1f\n", id, angle, speed);
    }

    if (cmd() == "/mg_status") {
        LOG_PRINTF("[MG] Shoulder: temp=%d torq=%d spd=%d enc=%u %s\n",
            shoulder_fb.temperature, shoulder_fb.torque, shoulder_fb.speed, shoulder_fb.encoder, shoulder_fb.valid ? "OK" : "FAIL");
        LOG_PRINTF("[MG] Elbow:    temp=%d torq=%d spd=%d enc=%u %s\n",
            elbow_fb.temperature, elbow_fb.torque, elbow_fb.speed, elbow_fb.encoder, elbow_fb.valid ? "OK" : "FAIL");
        LOG_PRINTF("[MG] Lift:     temp=%d torq=%d spd=%d enc=%u %s\n",
            lift_fb.temperature, lift_fb.torque, lift_fb.speed, lift_fb.encoder, lift_fb.valid ? "OK" : "FAIL");
    }

    if (cmd() == "/mg_enc") {
        float s_deg = mg_normalize_encoder(shoulder_fb.encoder);
        float e_deg = mg_normalize_encoder(elbow_fb.encoder);
        float l_deg = mg_normalize_encoder(lift_fb.encoder);
        LOG_PRINTF("[MG ENC RAW] Shoulder=%u  Elbow=%u  Lift=%u\n", shoulder_fb.encoder, elbow_fb.encoder, lift_fb.encoder);
        LOG_PRINTF("[MG ENC DEG] Shoulder=%.2f  Elbow=%.2f  Lift=%.2f  (k=%.1f ticks/deg)\n", s_deg, e_deg, l_deg, MG_TICKS_PER_DEG);
    }

    // ── 4. СМЫЧОК И ТРАЕКТОРИЯ ──
    if (cmd() == "/bow") {
        int dir = getInt(1);
        unsigned long dur = getInt(2);
        float spd = getFloat(3) / 100.0;
        bow_start_cmd(dir, dur, spd);
        LOG_PRINTF("[BOW CMD] Started! dir: %d, time: %lu ms, speed: %.2f, current_pos: %.2f\n", dir, dur, spd, bow_position);
    }

    if (cmd() == "/bow_loop") {
        bow_start_loop(getInt(1), getFloat(2) / 100.0);
    }

    if (cmd() == "/bow_stop") {
        bow_active = false;
        bow_looping = false;
    }

    if (cmd() == "/bow_p") {
        float v = getFloat(1);
        if (v >= 1.0f) {
            bow_p_factor = v;
            LOG_PRINTF("[BOW] S-curve factor set to: %.2f\n", bow_p_factor);
        } else {
            LOG_PRINTF("[BOW] ERROR: factor must be >= 1.0 (got %.2f)\n", v);
        }
    }

    if (cmd() == "/set_p0") {
        if (_argCount >= 3) {
            P0_X = getFloat(1);
            P0_Y = getFloat(2);
            LOG_PRINTF("[BOW] Точка P0 обновлена: X=%.1f, Y=%.1f\n", P0_X, P0_Y);
            if (!bow_active && bow_position <= 0.01f) {
                arm_set_target(P0_X, P0_Y, 30.0f, 30.0f);
            }
        } else {
            LOG_PRINTF("[BOW] ОШИБКА: Формат: /set_p0;<X>;<Y>\n");
        }
    }

    if (cmd() == "/set_p1") {
        if (_argCount >= 3) {
            P1_X = getFloat(1);
            P1_Y = getFloat(2);
            LOG_PRINTF("[BOW] Точка P1 обновлена: X=%.1f, Y=%.1f\n", P1_X, P1_Y);
            if (!bow_active && bow_position >= 0.99f) {
                arm_set_target(P1_X, P1_Y, 30.0f, 30.0f);
            }
        } else {
            LOG_PRINTF("[BOW] ОШИБКА: Формат: /set_p1;<X>;<Y>\n");
        }
    }

    if (cmd() == "/get_p") {
        LOG_PRINTF("[BOW] Текущая траектория: P0(%.1f, %.1f) ---> P1(%.1f, %.1f)\n", P0_X, P0_Y, P1_X, P1_Y);
    }

    // ── 5. ВЫСОТА ЛИФТА И ДИНАМИЧЕСКИЙ ПРИЖИМ ──
    if (cmd() == "/set_lift") {
        float angle = getFloat(1);
        arm_set_lift(angle, lift_string_speed); 
        LOG_PRINTF("[LIFT] Direct set to: %.1f\n", angle);
    }

    if (cmd() == "/lift_mg_set") {
        float angle = getFloat(1);
        float speed = getFloat(2);
        arm_set_lift(angle, speed);
        LOG_PRINTF("[LIFT] Target angle set to: %.1f, speed: %.1f\n", angle, speed);
    }

    if (cmd() == "/lift_speed") {
        if (_argCount >= 2) {
            float new_speed = getFloat(1);
            if (new_speed > 0) {
                lift_string_speed = new_speed;
                LOG_PRINTF("[LIFT] Transition speed changed to: %.1f\n", lift_string_speed);
            }
        } else {
            LOG_PRINTF("[LIFT] Current transition speed: %.1f\n", lift_string_speed);
        }
    }

    if (cmd() == "/set_lift_str") {
        if (_argCount >= 4) {
            int str_num = getInt(1);
            float angle_p0 = getFloat(2);
            float angle_p1 = getFloat(3);

            if (str_num >= 1 && str_num <= 4) {
                int str_idx = str_num - 1;
                lift_string_p0[str_idx] = angle_p0;
                lift_string_p1[str_idx] = angle_p1;
                LOG_PRINTF("[LIFT] Струна %d: начало (P0) = %.2f, конец (P1) = %.2f\n", str_num, angle_p0, angle_p1);
                
                if (sn_string == str_num) {
                    float p = ease(bow_position);
                    float target_lift = lift_string_p0[str_idx] + (lift_string_p1[str_idx] - lift_string_p0[str_idx]) * p;
                    arm_set_lift(target_lift, 20.0f); 
                }
            } else {
                LOG_PRINTF("[LIFT] ОШИБКА: Струна должна быть от 1 до 4!\n");
            }
        } else {
            LOG_PRINTF("[LIFT] ОШИБКА: Формат: /set_lift_str;<струна>;<угол_P0>;<угол_P1>\n");
        }
    }

    if (cmd() == "/get_lift_str") {
        LOG_PRINTF("[LIFT] Текущие настройки высоты струн (P0 -> P1):\n");
        for (int i = 0; i < 4; i++) {
            LOG_PRINTF("Струна %d: %.2f -> %.2f\n", i + 1, lift_string_p0[i], lift_string_p1[i]);
        }
    }

    if (cmd() == "/lift_to") {
        unsigned int ms = (unsigned int)getFloat(1);
        if (ms < 50) ms = 50;
        lift_arrival_timeout_ms = ms;
        LOG_PRINTF("[LIFT] Timeout = %u ms\n", lift_arrival_timeout_ms);
    }

    if (cmd() == "/dyn_press") {
        if (_argCount >= 2) {
            int state = getInt(1);
            dynamic_pressure_enabled = (state == 1);
            LOG_PRINTF("[BOW PRESS] Динамический режим %s\n", dynamic_pressure_enabled ? "ВКЛЮЧЕН" : "ВЫКЛЮЧЕН");
        } else {
            LOG_PRINTF("[BOW PRESS] ОШИБКА: Формат: /dyn_press;1 (ВКЛ) или /dyn_press;0 (ВЫКЛ)\n");
        }
    }

    if (cmd() == "/set_dyn_press") {
        if (_argCount >= 5) {
            int str_num = getInt(1);
            int p0_val  = getInt(2);
            int mid_val = getInt(3);
            int p1_val  = getInt(4);

            if (str_num >= 1 && str_num <= 4) {
                int idx = str_num - 1;
                bow_pressure_p0[idx]  = constrain(p0_val, 40, 162);
                bow_pressure_mid[idx] = constrain(mid_val, 40, 162);
                bow_pressure_p1[idx]  = constrain(p1_val, 40, 162);
                LOG_PRINTF("[BOW PRESS] Струна %d настроена: P0=%d, Центр=%d, P1=%d\n", str_num, bow_pressure_p0[idx], bow_pressure_mid[idx], bow_pressure_p1[idx]);
            } else {
                LOG_PRINTF("[BOW PRESS] ОШИБКА: Струна должна быть от 1 до 4!\n");
            }
        } else {
            LOG_PRINTF("[BOW PRESS] ОШИБКА: Формат: /set_dyn_press;<струна>;<P0>;<Центр>;<P1>\n");
        }
    }

    if (cmd() == "/get_dyn_press") {
        LOG_PRINTF("[BOW PRESS] Статус динамического режима: %s\n", dynamic_pressure_enabled ? "ВКЛЮЧЕН" : "ВЫКЛЮЧЕН");
        LOG_PRINTF("Текущие углы (P0 -> Центр -> P1):\n");
        for (int i = 0; i < 4; i++) {
            LOG_PRINTF("Струна %d: %d -> %d -> %d\n", i + 1, bow_pressure_p0[i], bow_pressure_mid[i], bow_pressure_p1[i]);
        }
    }

    if (cmd() == "/bow_press") {
        int target = getInt(1);
        bow_pressure_target = constrain(target, 0, 180); 
        bow_pressure_speed_time = (_argCount >= 3) ? getInt(2) : 0;
        LOG_PRINTF("[BOW PRESS] Угол: %d, Задержка: %d мс\n", bow_pressure_target, bow_pressure_speed_time);
    }

    // ── 6. НОТЫ, ПАЛЬЦЫ И СЕРВА СКРИПКИ ──
    if (cmd() == "/sn") {
        int press = (_argCount >= 4) ? getInt(3) : 1;
        set_string_and_note(getInt(1), getInt(2), press);
    }

    if (cmd() == "/psn") {
        psn_start(getInt(1), getInt(2), getInt(3));
    }

    if (cmd() == "/finger") {
        activate_finger(getInt(1));
    }

    if (cmd() == "/set_note") {
        int str_idx = sn_string - 1; 
        if (str_idx < 0) str_idx = 0;
        if (str_idx > 3) str_idx = 3;
        set_note(getInt(1), x_note[str_idx]);
    }

    if (cmd() == "/set_fingers") {
        if (_argCount >= 4) {
            int f_idx = getInt(1) - 1;
            int angle_up = getInt(2);
            int angle_down = getInt(3);
            
            if (f_idx >= 0 && f_idx < SERVO_COUNT) {
                finger_up[f_idx] = constrain(angle_up, 0, 180);
                finger_down[f_idx] = constrain(angle_down, 0, 180);
                pwm.setPWM(f_idx, 0, angle_to_pulse(finger_up[f_idx]));
                LOG_PRINTF("[FINGERS] Палец %d настроен: ПОДНЯТ = %d, НАЖАТ = %d\n", f_idx + 1, finger_up[f_idx], finger_down[f_idx]);
            } else {
                LOG_PRINTF("[FINGERS] ОШИБКА: Номер пальца должен быть от 1 до 4!\n");
            }
        } else {
            LOG_PRINTF("[FINGERS] ОШИБКА: Формат: /set_fingers;<номер>;<UP>;<DOWN>\n");
        }
    }

    if (cmd() == "/get_fingers") {
        LOG_PRINTLN("[FINGERS] Текущие настройки углов (Поднят -> Нажат):");
        for (int i = 0; i < SERVO_COUNT; i++) {
            LOG_PRINTF("Палец %d: %d -> %d\n", i + 1, finger_up[i], finger_down[i]);
        }
    }

    if (cmd() == "/set_mm") {
        if (_argCount >= 4) {
            int str_num = getInt(1);
            int note_num = getInt(2);
            float new_mm = getFloat(3);

            if (str_num >= 1 && str_num <= 4 && note_num >= 1 && note_num <= 7) {
                int str_idx = str_num - 1;
                int note_idx = note_num - 1;
                x_note[str_idx][note_idx] = (long)(new_mm * pulses_per_mm);
                LOG_PRINTF("[FINGERS] Струна %d, Нота %d установлена на %.2f мм (%ld шагов)\n", str_num, note_num, new_mm, x_note[str_idx][note_idx]);
            } else {
                LOG_PRINTF("[FINGERS] ОШИБКА: Струна должна быть 1-4, Нота 1-7!\n");
            }
        } else {
            LOG_PRINTF("[FINGERS] ОШИБКА: Использование: /set_mm;<струна>;<нота>;<миллиметры>\n");
        }
    }

    if (cmd() == "/get_mm") {
        LOG_PRINTF("[FINGERS] Текущая карта нот (в миллиметрах):\n");
        for (int s = 0; s < 4; s++) {
            LOG_PRINTF("Струна %d: ", s + 1);
            for (int n = 0; n < 7; n++) {
                float mm = (float)x_note[s][n] / pulses_per_mm;
                LOG_PRINTF("%.2f ", mm);
            }
            LOG_PRINTLN("");
        }
    }

    if (cmd() == "/violin") {
        set_violin_angle(getInt(1));
        if (_argCount >= 3) {
            violin_speed_time = getInt(2);
        }
    }

    if (cmd() == "/set_v_str") {
        if (_argCount >= 3) {
            int str_num = getInt(1);
            int angle = getInt(2);
            if (str_num >= 1 && str_num <= 4) {
                violin_string[str_num - 1] = constrain(angle, 0, 56); 
                LOG_PRINTF("[VIOLIN] Струна %d настроена на угол %d\n", str_num, violin_string[str_num - 1]);
                set_violin_string(str_num); 
            } else {
                LOG_PRINTF("[VIOLIN] ОШИБКА: Струна должна быть от 1 до 4!\n");
            }
        } else {
            LOG_PRINTF("[VIOLIN] ОШИБКА: Формат: /set_v_str;<струна 1-4>;<угол>\n");
        }
    }

    // ── 7. ИБТ-2, ПРИВОД И СЕНСОРЫ ──
    if (cmd() == "/lift_up") {
        lift_up((_argCount >= 2) ? getInt(1) : lift_speed);
    }

    if (cmd() == "/lift_down") {
        if (_argCount >= 3) {
            lift_down(getInt(1), getInt(2));
        } else if (_argCount >= 2) {
            lift_down(getInt(1), lift_down_duration);
        } else {
            lift_down(lift_speed, lift_down_duration);
        }
    }

    if (cmd() == "/lift_stop") {
        lift_stop();
    }

    if (cmd() == "/sensor_status") {
        bool sensor = digitalRead(SENSOR_MOTOR);
        LOG_PRINTF("[SENSOR] pin %d = %s  ->  скрипка %s\n",
            SENSOR_MOTOR, sensor ? "HIGH" : "LOW", lift_is_up ? "ПОДНЯТА" : "ОПУЩЕНА");
    }

    if (cmd() == "/up_to_play") {
        float bow_angle      = (_argCount >= 2) ? getFloat(1) : 90.0; 
        float mg_speed       = (_argCount >= 3) ? getFloat(2) : 17.0;
        unsigned long dur_ms = (_argCount >= 4) ? getInt(3) : 8000;
        play_go_up(bow_angle, mg_speed, dur_ms);
    }

    if (cmd() == "/down_to_play") {
        float bow_angle      = (_argCount >= 2) ? getFloat(1) : 20.0; 
        float mg_speed       = (_argCount >= 3) ? getFloat(2) : 17.0;
        unsigned long dur_ms = (_argCount >= 4) ? getInt(3) : 8000;
        play_go_down(bow_angle, mg_speed, dur_ms);
    }

    // ── 8. КАЛИБРОВКА, ПАУЗА И ПЛЕЕР ──
    if (cmd() == "/calibrate") {
        calibrate();
    }

    if (cmd() == "/cal") {
        calibrate_start();
    }

    if (cmd() == "/p") {
        pause_start(getInt(1));
    }

    if (cmd() == "/load_test") {
        queue_load_start();
    }

    if (cmd() == "/end") {
        queue_load_end();
    }

    if (cmd() == "/play") {
        queue_play();
    }

    if (cmd() == "/stop") {
        queue_stop();
    }

    // sound 
    if (cmd() == "/yes") {
        beep_yes_long();
        LOG_PRINTLN("[AUDIO] Signal: YES");
    }

    if (cmd() == "/no") {
        beep_no_long();
        LOG_PRINTLN("[AUDIO] Signal: NO");
    }
}




// ╔══════════════════════════════════════════════╗
// ║            COMMAND INPUTS                     ║
// ╚══════════════════════════════════════════════╝

// ── USB serial input ─────────────────
void handleUSB() {
    if (!Serial.available()) return;
    String input = Serial.readString();
    input.trim();

    if (loading && input != "/end") {
        queue_add(input);
    } else {
        executeCommand(input);
    }
}

// ── Bluetooth input ──────────────────
void handleBT() {
    if (!SerialBT.available()) return;
    String input = SerialBT.readStringUntil('\n');
    input.trim();

    if (loading && input != "/end") {
        queue_add(input);
    } else {
        executeCommand(input);
    }
}

// ╔══════════════════════════════════════════════╗
// ║            ESP-NOW                           ║
// ╚══════════════════════════════════════════════╝

// ── Send command to second ESP32 ─────
void sendToFlipper(String msg) {
    esp_now_send(flipperMAC, (uint8_t*)msg.c_str(), msg.length());
}

// ── Receive from second ESP32 ────────
void onESPNowReceive(const uint8_t *mac, const uint8_t *data, int len) {
    // 🔴 FIX: Race condition protected via FreeRTOS mutex
    portENTER_CRITICAL_ISR(&espnow_mux);
    int copyLen = len < 250 ? len : 250;
    memcpy(espnow_cmd_buf, data, copyLen);
    espnow_cmd_buf[copyLen] = '\0';
    has_espnow_cmd = true;
    portEXIT_CRITICAL_ISR(&espnow_mux);
}

// ── Handle ESP-NOW command ───────────
void handleESPNow() {
    // 🔴 FIX: Safely read the buffer synchronously
    if (has_espnow_cmd) {
        String cmd;
        portENTER_CRITICAL(&espnow_mux);
        cmd = String(espnow_cmd_buf);
        has_espnow_cmd = false;
        portEXIT_CRITICAL(&espnow_mux);

        executeCommand(cmd);
    }
}



void handleMega() {
    if (!MegaSerial.available()) return;
    String input = MegaSerial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;
    LOG_PRINTLN("[MEGA] <- " + input);
    executeCommand(input);
}





// ── Плеер очереди ────────────────────
void queue_update() {
    if (!playing) return;

    // очередь закончилась
    if (queue_index >= queue_count) {
        playing = false;
        LOG_PRINTLN("[QUEUE] Finished");
        return;
    }

    // ждём, пока предыдущая команда полностью отыграет
    if (!all_idle()) return;

    // запускаем следующую команду
    executeCommand(String(queue_cmd[queue_index]));
    queue_index++;
}





// ╔══════════════════════════════════════════════╗
// ║            SETUP                             ║
// ╚══════════════════════════════════════════════╝

void setup() {

    Serial.begin(115200);

// ── Загрузка настроек из NVS памяти ──
    prefs.begin("emma_cfg", false); // Открываем папку emma_cfg. false = можно и читать, и писать.

    // Проверяем, есть ли сохраненные данные (ориентируемся по ключу "f_up")
    if (prefs.getBytesLength("f_up") > 0) {
        // Массивы загружаем через getBytes
        prefs.getBytes("f_up", finger_up, sizeof(finger_up));
        prefs.getBytes("f_down", finger_down, sizeof(finger_down));
        
        prefs.getBytes("x_note", x_note, sizeof(x_note));
        prefs.getBytes("phys_fing", physical_fingers, sizeof(physical_fingers));
        prefs.getBytes("v_str", violin_string, sizeof(violin_string));
        
        prefs.getBytes("lift_p0", lift_string_p0, sizeof(lift_string_p0));
        prefs.getBytes("lift_p1", lift_string_p1, sizeof(lift_string_p1));
        
        prefs.getBytes("bp_p0", bow_pressure_p0, sizeof(bow_pressure_p0));
        prefs.getBytes("bp_mid", bow_pressure_mid, sizeof(bow_pressure_mid));
        prefs.getBytes("bp_p1", bow_pressure_p1, sizeof(bow_pressure_p1));

        // Координаты float загружаем через getFloat (второй аргумент — дефолтное значение, если ключа вдруг нет)
        P0_X = prefs.getFloat("P0_X", P0_X);
        P0_Y = prefs.getFloat("P0_Y", P0_Y);
        P1_X = prefs.getFloat("P1_X", P1_X);
        P1_Y = prefs.getFloat("P1_Y", P1_Y);

        LOG_PRINTLN("[SYS] Все переменные успешно загружены из памяти.");
    } else {
        LOG_PRINTLN("[SYS] Память пуста. Используются заводские настройки из кода.");
    }



    Serial.setTimeout(5);
    delay(500);
    Serial.printf("BOOT heap=%d\n", ESP.getFreeHeap());
    Serial.flush();

    // UART1 — MG Motors
    MotorSerial.begin(UART1_BAUD, SERIAL_8N1, UART1_RX, UART1_TX);
    // UART2 — Arduino Mega
    MegaSerial.begin(UART2_BAUD, SERIAL_8N1, UART2_RX, UART2_TX);
    MegaSerial.setTimeout(5);

    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
  

    // Light sensors
    pinMode(SENSOR_MOTOR,   INPUT);
    pinMode(SENSOR_STEPPER, INPUT);

    // lift motor
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);

    check_initial_lift_position(); // проверка начального состояния подьема скрипки 


    // === НОВАЯ ЛОГИКА: Выставление 3-го мотора при старте ===
    if (lift_is_up) {
        // Скрипка ПОДНЯТА. Выставляем рабочий/безопасный угол.
        // Замени 90.0 (угол) и 150.0 (скорость) на свои актуальные значения
        arm_set_lift(90.0, 30.0); 
        LOG_PRINTLN("[SYS] Startup: Violin UP -> Lift Target: 90.0");
    } else {
        // Скрипка ОПУЩЕНА. Выставляем парковочный угол.
        // Замени 0.0 (угол) и 150.0 (скорость) на свои актуальные значения
        arm_set_lift(20.0, 30.0);  
        LOG_PRINTLN("[SYS] Startup: Violin DOWN -> Lift Target: 0.0");
    }
    // ==========================================================


    // OLED
    display.setRotation(2);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.display();
    animateStartup();

    // PCA9685 (Servos)
    pwm.begin();
    pwm.setPWMFreq(PCA_FREQ);
    
    for (int i = 0; i < SERVO_COUNT; i++) {
        pwm.setPWM(i, 0, angle_to_pulse(finger_up[i]));
    }
    pwm.setPWM(VIOLIN_SERVO, 0, angle_to_pulse(VIOLIN_START));

    pwm.setPWM(BOW_PRESSURE_SERVO, 0, angle_to_pulse(BOW_PRESSURE_START));

    // Stepper motor
    stepper_begin(STEP_PIN, DIR_PIN, false);

    // Fingers mapping 
    //fingers_init(x_note, 7, pulses_per_mm);

    // Fingers mapping (инициализируем все 4 струны)
    // Fingers mapping 
    // ОСТОРОЖНО: Если мы загрузили x_note и physical_fingers из памяти,
    // они УЖЕ в шагах (pulses). Их нельзя умножать второй раз!
    if (prefs.getBytesLength("x_note") == 0) {
        // Загрузки не было, переводим базовые миллиметры в шаги мотора
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 7; j++) {
                // Умножаем на pulses_per_mm и округляем до ближайшего целого шага
                x_note[i][j] = (long)round(default_x_note_mm[i][j] * pulses_per_mm);
            }
            // Передаем множитель 1, так как мы УЖЕ перевели массив x_note в шаги
            fingers_init(x_note[i], 7, 1); 
        }
        for (int i = 0; i < 4; i++) {
            physical_fingers[i] *= pulses_per_mm;
        }
    } else {
        // Значения уже загружены из памяти в виде готовых шагов.
        // Просто инициализируем библиотеку с коэффициентом 1.
        for (int i = 0; i < 4; i++) {
            fingers_init(x_note[i], 7, 1); 
        }
    }

    // Bluetooth
    SerialBT.begin("EMMA");
    SerialBT.setTimeout(10);
    SerialBT.setPin("1234");

    // ESP-NOW
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(8);
    esp_now_init();
    esp_now_register_recv_cb(onESPNowReceive);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, flipperMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    // Логи приветствия переведены на макросы вывода
    LOG_PRINTLN("=== EMMA READY ===");
    LOG_PRINT("Free RAM: ");
    LOG_PRINT(ESP.getFreeHeap());
    LOG_PRINTLN(" bytes");

    calibrate();

    float start_m1, start_m2;
    calculate_ik(P0_X, P0_Y, start_m1, start_m2);
    motor_target_angle[0] = start_m1 + SHOULDER_OFFSET;
    motor_target_angle[1] = start_m2 + ELBOW_OFFSET;
    LOG_PRINTF("[SYS] Startup arm angles calculated for P0: Shoulder=%.2f, Elbow=%.2f\n", 
                motor_target_angle[0], motor_target_angle[1]);
}

// ╔══════════════════════════════════════════════╗
// ║            LOOP                              ║
// ╚══════════════════════════════════════════════╝

void loop() {
    // Handling communications
    handleUSB();
    handleBT();
    handleESPNow();
    handleMega();

    queue_update();  


    // Devices update
    fingers_update(); // Updated signature: no parameters needed
    violin_update();
    bow_pressure_update();
    dynamic_pressure_update();

    buzzer_update();


    calibrate_update();
    lift_update();  

    sn_update();
    

    bow_update();

    arm_trajectory_update(); // Считает геометрию для моторов 1 и 2, если взведен флаг движения


    psn_update();
    pause_update();


    if (automatic_loop_enabled) {
        motor_bus_update();      
    }

    //motor_bus_update();      // Крутит непрерывный бесконечный опрос шины для всех 3 моторов

    // Spinner on screen update every 500ms
    if (millis() - lastSpinner > 500) {
        lastSpinner = millis();
        spinnerIndex = (spinnerIndex + 1) % 4;
        drawMainScreen();
    }


}

///  /bow_loop;7500;100
/*
/bow;-1;1000;25

/bow;-1;1000;100

/bow;1;1000;100

/bow;-1;1000;50


/bow;-1;1000;25


/arm;400;200          — поехать в точку X=400 Y=200 (скорость 50)
/arm;400;200;100      — то же но скорость 100

/bow;5000             — смычок из P0 в P1 за 5 секунд
/bow;2000             — то же за 2 секунды

/mg_on                — включить моторы
/mg_off               — расслабить моторы
/mg_read              — считать углы и XY
/mg_test;90;100;1     — отправить угол 90, скорость 100, мотору 1
/mg_status            — feedback из struct (temp, torque, speed, encoder)

/set_note;3           — установить ноту 3
/finger;1             — нажать палец 1
/sn;1;1            — струна 2, нота 3

/violin130            — угол серво скрипки
/violin;30;20         — угол 30, скорость шага 20мс

/calibrate            — калибровка (блокирующая)
/cal                  — калибровка (неблокирующая)

/lift_up              — поднять скрипку
/lift_up;200          — поднять со скоростью 200
/lift_down            — опустить скрипку
/lift_down;200;3000   — опустить скорость 200, длительность 3с
/lift_stop            — стоп подъёма


/lift_mg_set;100;10
/lift_mg_set;95;10
*/



