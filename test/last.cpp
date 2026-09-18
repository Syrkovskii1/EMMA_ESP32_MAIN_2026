
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
#define VIOLIN_START 5

// ── IBT-2 Violin lift ────────────────
#define RPWM 25        
#define LPWM 26        

// ╔══════════════════════════════════════════════╗
// ║            MG MOTORS (Shoulder/Elbow/Lift)   ║
// ╚══════════════════════════════════════════════╝

// ── Arm link lengths (mm) ────────────
const float L1 = 290.0;
const float L2 = 310.0;

// ── Gear ratio and speed ─────────────
const float GEAR_RATIO_ARM  = 10.0;
const float GEAR_RATIO_LIFT = 36.0;

const float MAX_SPEED_SHOULDER = 450.0; // Подняли скорость для Плеча (ID 2)
const float MAX_SPEED_ELBOW    = 300.0; // Оставили базовую скорость для Локтя (ID 1)

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



// Массивы теперь строго на 3 элемента
const uint8_t MOTOR_IDS[3] = {SHOULDER_ID, ELBOW_ID, LIFT_ID};
float motor_target_angle[3] = {67.6, 141.9, 90};

float motor_target_speed[3] = {50, 50, 50};

// ── Offsets (degrees) ────────────────
const float SHOULDER_OFFSET = 90.0;
const float ELBOW_OFFSET = 90.0;
const float LIFT_OFFSET = 0.0; // Твой офсет для мотора подъема

// ── Trajectory Coordinates (mm) ──────
//const float P1_X = 327.0,  P1_Y = 325.0; 
const float P1_X = 298.0,  P1_Y = 305.0; 
const float P0_X = 538.0, P0_Y = 42.0;  

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
// REVERSED ARRAY: Note 1 is closest (18mm), Note 7 is furthest (110mm)
long x_note[7] = {18, 35, 52, 66, 80, 94, 110};

// ── Pulses per mm ────────────────────
int pulses_per_mm = 80;

// ── Servo angles: up / down ──────────
//    [0] Index
//    [1] Middle
//    [2] Ring
//    [3] Pinky
int finger_up[4]   = {45,  45,  5,   3};
int finger_down[4] = {0,   0,   50,  50};

int violin_string[4] = {33, 39, 45, 50};


// [0] - 1 струна, [1] - 2 струна, [2] - 3 струна, [3] - 4 струна
float lift_string[4] = {75.0, 82.5, 88.0, 98.0}; // ОБЯЗАТЕЛЬНО настрой эти углы под свою механику!

// Скорость (градусы в секунду), с которой рука переходит между струнами
float lift_string_speed = 100.0;


long CALIBR_DISL = 777;

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
// ── Независимая установка высоты руки (3 мотор) ──
void arm_set_lift(float angle_deg, float speed_dps) {
    motor_target_angle[2] = angle_deg + LIFT_OFFSET;
    motor_target_speed[2] = speed_dps;
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

bool mg_lift_arrived() {
    // lift_fb обновляется в motor_bus_update через round-robin
    return (abs(lift_fb.speed) <= 50);
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

float bow_smooth = 1.0;   // 1.0 = полный smoothstep, 0.0 = линейный

float ease(float t) {
    float lin = t;
    float smooth = t * t * (3.0 - 2.0 * t);
    return lin + (smooth - lin) * bow_smooth;
}

///////////////////////

float inv_ease(float p) {
    if (p <= 0.0f) return 0.0f;
    if (p >= 1.0f) return 1.0f;
    return 0.5f - sinf(asinf(1.0f - 2.0f * p) / 3.0f);
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

    if (dir == 0) {
        // авто-режим: программа сама меняет направление на краях
        bow_looping = true;
        bow_direction = (bow_position < 0.5f) ? 1 : -1;      // стартуем вперёд (или с того края, где ближе)
    } else {
        // обычный одиночный штрих
        bow_looping = false;
        bow_direction = dir;     // 1 или -1
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


// ── Bow update ───────────────────────
// ── Bow update ───────────────────────
void bow_update() {
    if (!bow_active) return;
    
    // Прерываем движение, если истек общий таймаут команды
    if (is_bow_timer(bow_timeout) == 1) {
        bow_active = false;
        bow_total_ms = millis() - bow_total_timer;
        LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms\n", last_command.c_str(), bow_total_ms);
        return;
    }
    
    // Считаем дельту времени (dt) в секундах
    unsigned long now = millis();
    float dt = (now - bow_last_time) / 1000.0f; 
    bow_last_time = now;
    
    // ФИКС: Меняем позицию и проверяем упоры ТОЛЬКО если время продвинулось вперед
    if (dt > 0.0f) {
        bow_position += bow_direction * bow_speed * dt;
        
        // Проверка верхнего программного упора
        if (bow_position >= 1.0f) { 
            bow_position = 1.0f; 
            if (bow_looping) {
                bow_direction = -1; // Разворот назад
            } else {
                bow_active = false; // Останов одиночного штриха
                bow_total_ms = millis() - bow_total_timer;
                LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms\n", last_command.c_str(), bow_total_ms);
            }
        }
        // Проверка нижнего программного упора
        else if (bow_position <= 0.0f) { 
            bow_position = 0.0f; 
            if (bow_looping) {
                bow_direction = 1;  // Разворот вперед
            } else {
                bow_active = false; // Останов одиночного штриха
                bow_total_ms = millis() - bow_total_timer;
                LOG_PRINTF("[BOW] cmd='%s' Total: %lu ms\n", last_command.c_str(), bow_total_ms);
            }
        }
    }
    
    // Расчет текущих координат по линии ведения сглаживания ease
    float p = ease(bow_position);
    float x = P0_X + (P1_X - P0_X) * p;
    float y = P0_Y + (P1_Y - P0_Y) * p;

    // Передаем координаты в неблокирующую математику ОЗК
    arm_set_target(x, y, MAX_SPEED_SHOULDER, MAX_SPEED_ELBOW);

    
}

// ╔══════════════════════════════════════════════╗
// ║            VIOLIN LIFT MOTOR                 ║
// ╚══════════════════════════════════════════════╝

int lift_speed = 250;
bool lifting_up = false;
bool lifting_down = false;
unsigned long lift_timer = 0;
unsigned long lift_down_duration = 5000;

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
        if (digitalRead(SENSOR_MOTOR) == HIGH) {
            lift_stop();
        } else if (is_lift_up_timer(lift_up_timeout) == 1) {
            lift_stop();
        }
    }

    if (lifting_down) {
        if (is_lift_timer(lift_down_duration) == 1) {
            lift_stop();
        }
    }
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
int violin_speed_time = 5;
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
// ║            STRING + NOTE (/sn)               ║
// ╚══════════════════════════════════════════════╝

int  sn_step = 0;          // 0=простой 1=поднять 2=таймер+ехать 3=ждать доезд 4=таймер+нажать
int  sn_string = 0;
int  sn_note = 0;
bool sn_press = true;
unsigned long sn_timer = 0;
unsigned long sn_press_timer = 0;
const uint16_t SN_LIFT_DELAY  = 30;  // мс на физический подъём пальца
const uint16_t SN_PRESS_DELAY = 100;  // мс паузы перед нажатием пальца
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

// ── /sn state machine ────────────────
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
            // шаг 2: ждём подъём серв, затем едем
            if (is_sn_timer(SN_LIFT_DELAY) == 1) {
                int violin_before = violin_current;         
                set_violin_string(sn_string);                 
                sn_string_changed = (violin_target != violin_before);  

                if (sn_string >= 1) {
                    if (sn_string <= 4) {
                        // Берем угол из нашего нового массива (индекс на 1 меньше номера струны)
                        float target_lift_angle = lift_string[sn_string - 1];
                        
                        // Отправляем неблокирующую команду в менеджер шины
                        arm_set_lift(target_lift_angle, lift_string_speed);
                    }
                }

                if (sn_note != 0) {
                    set_note(sn_note, x_note);
                }
                sn_step = 3;
            }
            break;

        case 3:

        
            // шаг 3: ждём пока скрипка довернулась И каретка доехала
          //   if (is_violin_ready() && !fingers_are_busy() && mg_lift_arrived()) {
          if (is_violin_ready() && !fingers_are_busy() ) {

                reset_sn_press_timer();    // взвести таймер паузы перед нажатием
                sn_step = 4;
            }
            break;

        case 4:
            // шаг 4: ждём паузу (только если струна менялась), затем нажимаем
            {
                uint16_t wait;
                if (sn_string_changed) {
                    wait = SN_PRESS_DELAY;   // струна менялась → 150
                } else {
                    wait = 0;                // струна не менялась → 0
                }
                if (is_sn_press_timer(wait) == 1) {
                    if (sn_note != 0 && sn_press) {
                        activate_finger(get_active_finger());
                    }
                    
                    sn_total_ms = millis() - sn_start_ms;
                    LOG_PRINTF("[SN] cmd='%s' Total: %lu ms\n", last_command.c_str(), sn_total_ms);
                    sn_step = 0;
                }
            }
            break;
    }
}

// ╔══════════════════════════════════════════════╗
// ║            CALIBRATION                       ║
// ╚══════════════════════════════════════════════╝

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

// ── Non-blocking calibration ─────────
int cal_state = 0;
bool calibrating = false;

void calibrate_start() {
    cal_state = 1;
    calibrating = true;
    stepper_stop();
    stepper_move(400, true, 4000, 1000, 50, 50);
}

void calibrate_update() {
    if (!calibrating) return;

    switch (cal_state) {
        case 1:
            if (stepper_is_ready()) {
                cal_state = 2;
                stepper_move(999999, false, 4000, 1000, 50, 50);
            }
            break;

        case 2:
            // Wait for the physical sensor to trigger (LOW = pressed)
            if (digitalRead(SENSOR_STEPPER) == LOW) {
                stepper_stop();
                cal_state = 3;
                stepper_move(CALIBR_DISL, true, 4000, 1000, 50, 50);
            }
            break;

        case 3:
            if (stepper_is_ready()) {
                // Sync internal coordinates directly!
                fingers_set_home(x_note);

                cal_state = 0;
                calibrating = false;
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

#define QUEUE_MAX  120          // максимум команд в очереди

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


bool all_idle() {
  //  return (sn_step == 0) && !bow_active && !fingers_are_busy() && mg_motors_arrived();
   // return (sn_step == 0) && !bow_active && !fingers_are_busy();
    //return (sn_step == 0) && !bow_active && !fingers_are_busy() && !pause_active;

     return (sn_step == 0) && !bow_active && !fingers_are_busy() && !pause_active && !psn_active;
}




// ╔══════════════════════════════════════════════╗
// ║            COMMAND EXECUTOR                   ║
// ╚══════════════════════════════════════════════╝


void executeCommand(String input) {
    parse(input);
    last_command = input;  
    showOnScreen(input);
    
    if (cmd() == "/load_test") {
        queue_load_start();
    }

    if (cmd() == "/mega") {
        send_to_mega(_args[1]);
    }


    if (cmd() == "/bow_smooth") {
        float v = getFloat(1);
        if (v >= 0.0 && v <= 1.0) {
            bow_smooth = v;
            LOG_PRINTF("[BOW] smooth set to: %.2f\n", bow_smooth);
        } else {
            LOG_PRINTF("[BOW] smooth ignored: %.2f (use 0.0..1.0)\n", v);
        }
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

    if (cmd() == "/set_note") {
        if (_argCount >= 3) {
            set_note(getInt(1), x_note);
        } else {
            set_note(getInt(1), x_note);
        }
    }

    if (cmd() == "/p") {
    pause_start(getInt(1));
    }


    if (cmd() == "/finger") {
        activate_finger(getInt(1));
    }
    if (cmd() == "/calibrate") {
        calibrate();
    }
    if (cmd() == "/cal") {
        calibrate_start();
    }
    if (cmd() == "/violin") {
        set_violin_angle(getInt(1));
        if (_argCount >= 3) {
            violin_speed_time = getInt(2);
        }
    }
    if (cmd() == "/sn") {
        int press = 1;                       // по умолчанию — нажимать
        if (_argCount >= 4) press = getInt(3);
        set_string_and_note(getInt(1), getInt(2), press);
    }


    if (cmd() == "/bow_loop") {
        bow_start_loop(getInt(1), getFloat(2) / 100.0);
    }
    if (cmd() == "/bow_stop") {
        bow_active = false;
        bow_looping = false;
    }
    if (cmd() == "/mg_read") {
            bow_active = false;
            arm_moving = false;
            motor_busy = false;
            delay(10);
            while (MotorSerial.available()) MotorSerial.read();
            
            float a1 = mg_read_angle(SHOULDER_ID);
            float a2 = mg_read_angle(ELBOW_ID);
            float a3 = mg_read_angle(LIFT_ID); // Считываем Лифт
            
            float q1 = a1 - SHOULDER_OFFSET;
            float q2 = a2 - ELBOW_OFFSET;
            
            float x, y;
            calculate_fk(q1, q2, x, y);
            
            // Выводим все три мотора в лог через макрос
            LOG_PRINTF("[MG] Shoulder: %.2f  Elbow: %.2f  Lift: %.2f  X: %.1f  Y: %.1f\n", a1, a2, a3, x, y);
        }
    if (cmd() == "/mg_test") {
        float angle = getFloat(1);  // angle
        float speed = getFloat(2);  // speed
        uint8_t id = getInt(3);     // motor id
        
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

    if (cmd() == "/lift_mg_set") {
        float angle = getFloat(1);
        float speed = getFloat(2);
        arm_set_lift(angle, speed);
        LOG_PRINTF("[LIFT] Target angle set to: %.1f, speed: %.1f\n", angle, speed);
    }


    if (cmd() == "/lift_up") {
        if (_argCount >= 2) {
            lift_up(getInt(1));
        } else {
            lift_up(lift_speed);
        }
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
    if (cmd() == "/mg_on") {
            for (int i = 0; i < 3; i++) {
                set_motor_state_blocking(MOTOR_IDS[i], true);
                delay(5);
            }
        }
        if (cmd() == "/mg_off") {
            for (int i = 0; i < 3; i++) {
                set_motor_state_blocking(MOTOR_IDS[i], false);
                delay(5);
            }
        }
    if (cmd() == "/psn") {
    psn_start(getInt(1), getInt(2), getInt(3));
    }

    if (cmd() == "/bow") {
        int dir = getInt(1);
        unsigned long dur = getInt(2);
        float spd = getFloat(3) / 100.0;
        
        bow_start_cmd(dir, dur, spd);
        
        LOG_PRINTF("[BOW CMD] Started! dir: %d, time: %lu ms, speed: %.2f, current_pos: %.2f\n", 
                    dir, dur, spd, bow_position);
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

    if (cmd() == "/log_mode") {
        int newMode = getInt(1);
        if (newMode >= 1 && newMode <= 3) {
            currentLogMode = (LogRoute)newMode;
            
            // Робот подтвердит смену режима в тот канал, который сейчас активен
            LOG_PRINTF("[SYS] Log mode changed to: %d\n", newMode);
        } else {
            LOG_PRINTF("[SYS] ERROR! Unknown mode: %d. Use 1 (BT), 2 (USB), 3 (BOTH)\n", newMode);
        }
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


// ╔══════════════════════════════════════════════╗
// ║            ARDUINO MEGA                      ║
// ╚══════════════════════════════════════════════╝

void send_to_mega(String cmd) {
    MegaSerial.println(cmd);
    LOG_PRINTLN("[MEGA] -> " + cmd);
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
    delay(500);
    Serial.printf("BOOT heap=%d\n", ESP.getFreeHeap());
    Serial.flush();


    Serial.begin(115200);
    Serial.setTimeout(5);
    
    // UART1 — MG Motors
    MotorSerial.begin(UART1_BAUD, SERIAL_8N1, UART1_RX, UART1_TX);
    // UART2 — Arduino Mega
    MegaSerial.begin(UART2_BAUD, SERIAL_8N1, UART2_RX, UART2_TX);

    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
  

    // Light sensors
    pinMode(SENSOR_MOTOR,   INPUT);
    pinMode(SENSOR_STEPPER, INPUT);

    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);

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

    // Stepper motor
    stepper_begin(STEP_PIN, DIR_PIN, false);

    // Fingers mapping 
    fingers_init(x_note, 7, pulses_per_mm);

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
    calibrate_update();
    lift_update();  

    sn_update();
    

    bow_update();

    arm_trajectory_update(); // Считает геометрию для моторов 1 и 2, если взведен флаг движения


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

    psn_update();
    pause_update();
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



