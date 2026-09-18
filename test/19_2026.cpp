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
// ║            MG MOTORS (Shoulder/Elbow)        ║
// ╚══════════════════════════════════════════════╝

// ── Arm link lengths (mm) ────────────
const float L1 = 235.0;
const float L2 = 310.0;

// ── Gear ratio and speed ─────────────
const float GEAR_RATIO = 10.0;
const float MAX_SPEED_DPS = 50.0;

// ── Motor IDs ────────────────────────
const uint8_t SHOULDER_ID = 2;
const uint8_t ELBOW_ID = 1;

struct MotorFeedback {
    int8_t   temperature;
    int16_t  torque;
    int16_t  speed;
    uint16_t encoder;
    bool     valid;
    uint32_t timestamp_us;
};

MotorFeedback shoulder_fb;
MotorFeedback elbow_fb;

MotorFeedback last_fb;

// Неблокирующая отправка на моторы 
bool motor_busy = false;
uint32_t motor_send_time = 0;
uint8_t rx_buf[13];
int rx_count = 0;
uint8_t current_motor = 0;

const uint8_t MOTOR_IDS[2] = {SHOULDER_ID, ELBOW_ID};
float motor_target_angle[2] = {0, 0};
float motor_target_speed[2] = {0, 0};



// ── Offsets (degrees) ────────────────
const float SHOULDER_OFFSET = 90.0;
const float ELBOW_OFFSET = 90.0;

// ── Trajectory Coordinates (mm) ──────
const float P1_X = 310.0,  P1_Y = 290.0; // Start point


const float P0_X = 530.0, P0_Y = 30.0;  // End point

const float DT = 0.01;                  // 100 Hz update rate



//   /arm;310;290

//   /arm;530;30

float arm_target_x = P0_X;
float arm_target_y = P0_Y;
bool arm_moving = false;



// ── UART Definitions ─────────────────
#define MotorSerial  Serial1   // UART1 — MG Motors
#define MegaSerial   Serial2   // UART2 — Arduino Mega

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

// ── Bluetooth ────────────────────────
BluetoothSerial SerialBT;

// ── Note positions (mm) ──────────────
long x_note[7] = {113, 94, 80, 66, 51, 35, 18};

// ── Finger positions (filled in init) 
long fingers_x[4] = {0, 0, 0, 0};

// ── Pulses per mm ────────────────────
int pulses_per_mm = 40;

// ── Servo angles: up / down ──────────
//    [0] Index
//    [1] Middle
//    [2] Ring
//    [3] Pinky
int finger_up[4]   = {50,  50,  0,   0};
int finger_down[4] = {0,   0,   50,  50};

int violin_string[4] = {30, 35, 40, 46};
long CALIBR_DISL = 1400;

// ╔══════════════════════════════════════════════╗
// ║            OBJECTS                           ║
// ╚══════════════════════════════════════════════╝

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Spinner ──────────────────────────
const char spinnerFrames[] = {'|', '/', '-', '\\'};
int spinnerIndex = 0;
unsigned long lastSpinner = 0;

// ╔══════════════════════════════════════════════╗
// ║            MG MOTORS & TRAJECTORY            ║
// ╚══════════════════════════════════════════════╝
void parse_motor_response(uint8_t* buf, int len, MotorFeedback &fb) {
    // Checksum header (байты 0-3, сумма = байт 4)
    uint8_t h_sum = buf[0] + buf[1] + buf[2] + buf[3];
    if (h_sum != buf[4]) {
        fb.valid = false;
        return;
    }
    
    // Checksum data (байты 5 до len-2, сумма = последний байт)
    uint8_t d_sum = 0;
    for (int i = 5; i < len - 1; i++) d_sum += buf[i];
    if (d_sum != buf[len - 1]) {
        fb.valid = false;
        return;
    }
    
    // Парсим данные (только для ответа на 0xA4, 13 байт)
    if (len == 13) {
        fb.temperature = (int8_t)buf[5];
        memcpy(&fb.torque,  &buf[6], 2);
        memcpy(&fb.speed,   &buf[8], 2);
        memcpy(&fb.encoder, &buf[10], 2);
    }
    
    fb.valid = true;
    fb.timestamp_us = micros();
}

void beep_error() {
    tone(BUZZER_PIN, 2000, 50);  // 2кГц, 100мс
}

bool check_motor_limit(uint8_t motor_id, float angle) {
    if (motor_id == SHOULDER_ID && (angle < 67.0 || angle > 103.0)) {
        Serial.printf("[ARM] BLOCKED! Shoulder %.1f out of [67..103]\n", angle);
        beep_error();
        return false;
    }
    if (motor_id == ELBOW_ID && (angle < 90.0 || angle > 180.0)) {
        Serial.printf("[ARM] BLOCKED! Elbow %.1f out of [90..180]\n", angle);
        beep_error();
        return false;
    }
    return true;
}

void send_cmd_a4(uint8_t motor_id, float angle_deg, float speed_dps) {
    if (!check_motor_limit(motor_id, angle_deg)) return;  // защита 

    int64_t angle_val = (int64_t)(angle_deg * GEAR_RATIO * 100.0);
    uint32_t speed_val = (uint32_t)(fabs(speed_dps) * GEAR_RATIO * 100.0);
    
    uint8_t buf[18];
    buf[0] = 0x3E; buf[1] = 0xA4; buf[2] = motor_id; buf[3] = 0x0C;
    buf[4] = (buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF;
    
    memcpy(&buf[5], &angle_val, 8);
    memcpy(&buf[13], &speed_val, 4);
    
    uint8_t data_sum = 0;
    for (int i = 5; i < 17; i++) data_sum += buf[i];
    buf[17] = data_sum;
    
    // Очищаем входной буфер
    while (MotorSerial.available()) MotorSerial.read();
    
    // Отправляем и всё, не ждём ответа
    MotorSerial.write(buf, 18);
    
    motor_busy = true;
    rx_count = 0;
    motor_send_time = micros();
}


void send_cmd_a4_blocking(uint8_t motor_id, float angle_deg, float speed_dps) {

    if (!check_motor_limit(motor_id, angle_deg)) return;  // защита 

    int64_t angle_val = (int64_t)(angle_deg * GEAR_RATIO * 100.0);
    uint32_t speed_val = (uint32_t)(fabs(speed_dps) * GEAR_RATIO * 100.0);
    
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

void calculate_ik(float x, float y, float &m1_deg, float &m2_deg) {
    float d = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    if (d < -1.0f) d = -1.0f;
    if (d > 1.0f) d = 1.0f;
    
    float q2 = acos(d);
    float q1 = atan2(y, x) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));
    
    m1_deg = q1 * 180.0f / PI; 
    m2_deg = q2 * 180.0f / PI;
}

void calculate_fk(float q1_deg, float q2_deg, float &x, float &y) {
    float q1 = q1_deg * PI / 180.0f;
    float q2 = q2_deg * PI / 180.0f;
    x = L1 * cos(q1) + L2 * cos(q1 + q2);
    y = L1 * sin(q1) + L2 * sin(q1 + q2);
}


void arm_set_target(float x, float y, float speed) {
    arm_target_x = x;
    arm_target_y = y;
    arm_moving = true;
    motor_target_speed[0] = speed;
    motor_target_speed[1] = speed;
}

void arm_continuous_update() {
    if (motor_busy) {
        MotorFeedback &fb = (current_motor == 0) ? shoulder_fb : elbow_fb;
        
        if (motor_receive(fb)) {
            current_motor++;
            if (current_motor < 2) {
                send_cmd_a4(MOTOR_IDS[current_motor],
                           motor_target_angle[current_motor],
                           motor_target_speed[current_motor]);
            } else {
                current_motor = 0;
                arm_moving = false;
            }
        }
        return;
    }
    
    if (!arm_moving) return;
    
    // IK
    float m1_raw, m2_raw;
    calculate_ik(arm_target_x, arm_target_y, m1_raw, m2_raw);
    
    motor_target_angle[0] = m1_raw + SHOULDER_OFFSET;
    motor_target_angle[1] = m2_raw + ELBOW_OFFSET;
    motor_target_speed[0] = MAX_SPEED_DPS;
    motor_target_speed[1] = MAX_SPEED_DPS;
    
    current_motor = 0;
    send_cmd_a4(MOTOR_IDS[0],
               motor_target_angle[0],
               motor_target_speed[0]);
}



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
    Serial.printf("[MG] Motor %d: %s\n", motor_id, state ? "ON" : "OFF");
}

float mg_read_angle(uint8_t motor_id) {
    uint8_t cmd[5] = {0x3E, 0x92, motor_id, 0x00, 0};
    cmd[4] = (cmd[0] + cmd[1] + cmd[2] + cmd[3]) & 0xFF;
    
    // Блокирующая — только для разовых запросов
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
    
    // Угол в байтах 5-12 (int64_t), единица = 0.01° * gear_ratio
    int64_t angle_raw = 0;
    memcpy(&angle_raw, &resp[5], 8);
    
    return (float)angle_raw / (GEAR_RATIO * 100.0);
}

unsigned long bow_timer = 0;
unsigned long bow_last_time = 0; 
unsigned long bow_timeout = 0;
float bow_position = 0.5; // Starting in the middle, keeps its position after stopping
float bow_speed = 0.0;
int bow_direction = 1;
bool bow_active = false;

void reset_bow_timer() { bow_timer = millis(); }

byte is_bow_timer(unsigned long p_t) {
    return (millis() - bow_timer >= p_t) ? 1 : 0;
}

void bow_start_cmd(int dir, unsigned long duration_ms, float speed) {
    bow_direction = dir;
    bow_timeout = duration_ms;
    bow_speed = speed; 
    bow_active = true;
    
    reset_bow_timer();
    bow_last_time = millis(); // Init dt timer
}

void bow_update() {
    if (!bow_active) return;
    
    // Stop if duration timeout reached
    if (is_bow_timer(bow_timeout) == 1) {
        bow_active = false;
        return;
    }
    
    // Calculate Delta Time (dt) in seconds
    unsigned long now = millis();
    float dt = (now - bow_last_time) / 1000.0; 
    bow_last_time = now;
    
    // Update position (S = V * t)
    bow_position += bow_direction * bow_speed * dt;
    
    // Physical hardware limits
    if (bow_position >= 1.0) { 
        bow_position = 1.0; 
        bow_active = false; 
    }
    if (bow_position <= 0.0) { 
        bow_position = 0.0; 
        bow_active = false; 
    }
    
    // Calculate target coordinates on the bow line
    float x = P0_X + (P1_X - P0_X) * bow_position;
    float y = P0_Y + (P1_Y - P0_Y) * bow_position;
    
    arm_set_target(x, y, MAX_SPEED_DPS);
}
// ╔══════════════════════════════════════════════╗
// ║            VIOLIN LIFT MOTOR                 ║
// ╚══════════════════════════════════════════════╝

int lift_speed = 250;
bool lifting_up = false;
bool lifting_down = false;
unsigned long lift_timer = 0;
unsigned long lift_down_duration = 5000;

void reset_lift_timer() { lift_timer = millis(); }

byte is_lift_timer(uint16_t p_t) {
    return (millis() - lift_timer >= p_t) ? 1 : 0;
}

unsigned long lift_up_timer = 0;
unsigned long lift_up_timeout = 6500;

void reset_lift_up_timer() { lift_up_timer = millis(); }

byte is_lift_up_timer(uint16_t p_t) {
    return (millis() - lift_up_timer >= p_t) ? 1 : 0;
}

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

void lift_update() {
    if (lifting_up) {
        if (digitalRead(SENSOR_MOTOR) == HIGH) {
            lift_stop();
            Serial.println("[LIFT] Sensor! Stop");
        } else if (is_lift_up_timer(lift_up_timeout) == 1) {
            lift_stop();
            Serial.println("[LIFT] UP Timeout! Stop");
        }
    }

    if (lifting_down) {
        if (is_lift_timer(lift_down_duration) == 1) {
            lift_stop();
            Serial.println("[LIFT] Timer! Stop");
        }
    }
}

// ╔══════════════════════════════════════════════╗
// ║            OLED SCREEN                       ║
// ╚══════════════════════════════════════════════╝

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
// ║            SERVOS                            ║
// ╚══════════════════════════════════════════════╝

int angle_to_pulse(int angle) {
    return map(angle, 0, 180, PULSE_MIN, PULSE_MAX);
}

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
int violin_speed_time = 30;
int violin_servo_step = 1;
int violin_target = VIOLIN_START;
int violin_current = VIOLIN_START;

void reset_violin_timer() { violin_timer = millis(); }

byte is_violin_timer(uint16_t p_t) {
    return (millis() - violin_timer >= p_t) ? 1 : 0;
}

void set_violin_angle(int angle) {
    // Protection limits up to 56
    violin_target = constrain(angle, 0, 56); 
}

bool is_violin_ready() {
    return !violin_servo_moving;
}

void violin_update() {
    if (violin_target != violin_current) {
        if (!violin_servo_moving) {
            violin_servo_moving = true;
            reset_violin_timer();
        }

        if (is_violin_timer(violin_speed_time) == 1) {
            reset_violin_timer();

            if (violin_target > violin_current) {
                violin_current += violin_servo_step;
                if (violin_current > violin_target) violin_current = violin_target;
            }

            if (violin_target < violin_current) {
                violin_current -= violin_servo_step;
                if (violin_current < violin_target) violin_current = violin_target;
            }
        }

        pwm.setPWM(VIOLIN_SERVO, 0, angle_to_pulse(violin_current));
    } else {
        violin_servo_moving = false;
    }
}

void set_violin_string(int stringNum) {
    if (stringNum >= 1 && stringNum <= 4) {
        set_violin_angle(violin_string[stringNum - 1]);
    }
}

// ── State machine string + note ──────
bool waiting_string_note = false;

void set_string_and_note(int stringNum, int nota) {
    activate_finger(-1);
    set_violin_string(stringNum);
    
    if (nota == 0) {
        waiting_string_note = false;
        return;
    }
    
    set_note(nota, x_note, fingers_x, false);
    waiting_string_note = true;
}

void string_note_update() {
    if (!waiting_string_note) return;

    if (is_violin_ready() && stepper_is_ready()) {
        activate_finger(get_active_finger());
        waiting_string_note = false;
    }
}

// ╔══════════════════════════════════════════════╗
// ║            CALIBRATION                       ║
// ╚══════════════════════════════════════════════╝

void calibrate() {
    Serial.println("[CAL] Moving away from sensor...");
    stepper_set_target(400, 4000, 1000, 50, 50);
    while (!stepper_is_ready()) delay(1);

    Serial.println("[CAL] Moving to sensor...");
    stepper_set_target(-999999, 4000, 1000, 50, 50);

    while (digitalRead(SENSOR_STEPPER) == HIGH) delay(1);

    stepper_stop();
    stepper_reset();

    Serial.println("[CAL] Sensor reached! Moving back...");

    stepper_set_target(CALIBR_DISL, 4000, 1000, 50, 50);
    while (!stepper_is_ready()) delay(1);

    stepper_reset();
    stepper_set_position(1400);
    Serial.println("[CAL] Calibration completed");
}

// ── Non-blocking calibration ─────────
int cal_state = 0;
bool calibrating = false;

void calibrate_start() {
    cal_state = 1;
    calibrating = true;
    Serial.println("[CAL] Moving away from sensor...");
    stepper_set_target(400, 4000, 1000, 50, 50);
}

void calibrate_update() {
    if (!calibrating) return;

    switch (cal_state) {
        case 1:
            if (stepper_is_ready()) {
                cal_state = 2;
                Serial.println("[CAL] Moving to sensor...");
                stepper_set_target(-999999, 4000, 1000, 50, 50);
            }
            break;

        case 2:
            if (digitalRead(SENSOR_STEPPER) == LOW) {
                stepper_stop();
                stepper_reset();
                cal_state = 3;
                Serial.println("[CAL] Sensor reached! Moving back...");
                stepper_set_target(CALIBR_DISL, 4000, 1000, 50, 50);
            }
            break;

        case 3:
            if (stepper_is_ready()) {
                stepper_reset();
                stepper_set_position(1400);
                cal_state = 0;
                calibrating = false;
                Serial.println("[CAL] Calibration completed");
            }
            break;
    }
}

// ╔══════════════════════════════════════════════╗
// ║            COMMAND PARSER                    ║
// ╚══════════════════════════════════════════════╝

String _args[7];
int _argCount = 0;

void parse(String input) {
    input.trim();
    _argCount = 0;
    int start = 0;
    while (start < (int)input.length() && _argCount < 7) {
        int sep = input.indexOf(';', start);
        if (sep == -1) sep = input.length();
        _args[_argCount++] = input.substring(start, sep);
        start = sep + 1;
    }
}

String cmd()       { return _args[0]; }
int    getInt(int i) { return _args[i].toInt(); }

void executeCommand(String input) {
    parse(input);
    showOnScreen(input);

    if (cmd() == "/set_note") {
        if (_argCount >= 3) {
            set_note(getInt(1), x_note, fingers_x, getInt(2));
        } else {
            set_note(getInt(1), x_note, fingers_x);
        }
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
        set_string_and_note(getInt(1), getInt(2));
    }

    if (cmd() == "/mg_read") {
        bow_active = false;
        arm_moving = false;
        motor_busy = false;
        delay(10);
        while (MotorSerial.available()) MotorSerial.read();
        
        float a1 = mg_read_angle(SHOULDER_ID);
        float a2 = mg_read_angle(ELBOW_ID);
        
        float q1 = a1 - SHOULDER_OFFSET;
        float q2 = a2 - ELBOW_OFFSET;
        
        float x, y;
        calculate_fk(q1, q2, x, y);
        
        Serial.printf("[MG] Shoulder: %.2f  Elbow: %.2f  X: %.1f  Y: %.1f\n", a1, a2, x, y);
    }


    if (cmd() == "/mg_test") {
        float angle = getInt(1);  // угол
        float speed = getInt(2);  // скорость
        uint8_t id = getInt(3);   // motor id
        
        send_cmd_a4_blocking(id, angle, speed);
        Serial.printf("[MG] Sent: id=%d angle=%.1f speed=%.1f\n", id, angle, speed);
    }

    if (cmd() == "/mg_status") {
        Serial.printf("[MG] Shoulder: temp=%d torq=%d spd=%d enc=%u %s\n",
            shoulder_fb.temperature, shoulder_fb.torque,
            shoulder_fb.speed, shoulder_fb.encoder,
            shoulder_fb.valid ? "OK" : "FAIL");
        Serial.printf("[MG] Elbow:    temp=%d torq=%d spd=%d enc=%u %s\n",
            elbow_fb.temperature, elbow_fb.torque,
            elbow_fb.speed, elbow_fb.encoder,
            elbow_fb.valid ? "OK" : "FAIL");
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
        set_motor_state_blocking(SHOULDER_ID, true);
        delay(5);
        set_motor_state_blocking(ELBOW_ID, true);
    }

    if (cmd() == "/mg_off") {
        set_motor_state_blocking(SHOULDER_ID, false);
        delay(5);
        set_motor_state_blocking(ELBOW_ID, false);
    }

// 1. UPDATE THIS IN YOUR executeCommand() FUNCTION:
    if (cmd() == "/bow") {
        int dir = getInt(1);
        unsigned long dur = getInt(2);
        float spd = getInt(3) / 100.0;
        
        bow_start_cmd(dir, dur, spd);
        
        // Print command parsing results to Serial
        Serial.printf("[BOW CMD] Started! dir: %d, time: %lu ms, speed: %.2f, current_pos: %.2f\n", 
                      dir, dur, spd, bow_position);
    }



    if (cmd() == "/arm") {
        float x = getInt(1);
        float y = getInt(2);
        float spd = 50.0;
        if (_argCount >= 4) spd = getInt(3);
        
        float m1_raw, m2_raw;
        calculate_ik(x, y, m1_raw, m2_raw);
        
        
        
        send_cmd_a4_blocking(SHOULDER_ID, m1_raw + SHOULDER_OFFSET, spd);
        send_cmd_a4_blocking(ELBOW_ID, m2_raw + ELBOW_OFFSET, spd);
        
        Serial.printf("[ARM] X=%.0f Y=%.0f S=%.1f E=%.1f\n", 
            x, y, m1_raw + SHOULDER_OFFSET, m2_raw + ELBOW_OFFSET);
    }
}

void handleUSB() {
    if (!Serial.available()) return;
    String input = Serial.readString();
    executeCommand(input);
}

void handleBT() {
    if (!SerialBT.available()) return;
    String input = SerialBT.readStringUntil('\n');
    input.trim();
    Serial.println("BT: " + input);
    executeCommand(input);
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
    char buf[251];
    int copyLen = len < 250 ? len : 250;
    memcpy(buf, data, copyLen);
    buf[copyLen] = '\0';
    Serial.print("ESPNOW: ");
    Serial.println(buf);
    executeCommand(String(buf));
}

// ╔══════════════════════════════════════════════╗
// ║            SETUP                             ║
// ╚══════════════════════════════════════════════╝

void setup() {
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
    stepper_begin(STEP_PIN, DIR_PIN);

    // Fingers
    fingers_init(x_note, 7, fingers_x, pulses_per_mm);

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

    Serial.println("=== EMMA READY ===");
    Serial.print("Free RAM: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    calibrate();

 
}

// ╔══════════════════════════════════════════════╗
// ║            LOOP                              ║
// ╚══════════════════════════════════════════════╝

void loop() {
    // Handling communications
    handleUSB();
    handleBT();

    // Devices update
    fingers_update(fingers_x);
    violin_update();
    calibrate_update();
    lift_update();
    string_note_update();
        
    bow_update();

    arm_continuous_update();

    // Spinner on screen update every 500ms
    if (millis() - lastSpinner > 500) {
        lastSpinner = millis();
        spinnerIndex = (spinnerIndex + 1) % 4;
        drawMainScreen();
    }
}

// /bow;5000

/*
/bow;-1;1000;25

/bow;-1;1000;50

/bow;1;1000;25

/bow;1;1000;50


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
/sn;2;3               — струна 2, нота 3

/violin;30            — угол серво скрипки
/violin;30;20         — угол 30, скорость шага 20мс

/calibrate            — калибровка (блокирующая)
/cal                  — калибровка (неблокирующая)

/lift_up              — поднять скрипку
/lift_up;200          — поднять со скоростью 200
/lift_down            — опустить скрипку
/lift_down;200;3000   — опустить скорость 200, длительность 3с
/lift_stop            — стоп подъёма


*/