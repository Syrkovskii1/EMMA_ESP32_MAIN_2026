

/*
#include <Arduino.h>
#include "EMMA_Stepper_Motor.h"
#include "EMMA_Fingers.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

#include "BluetoothSerial.h"


// ╔══════════════════════════════════════════════╗
// ║           ОСНОВНЫЕ НАСТРОЙКИ                 ║
// ╚══════════════════════════════════════════════╝

// ── I2C ──────────────────────────────
#define SDA_PIN  21
#define SCL_PIN  22

// ── Пины мотора ──────────────────────
#define STEP_PIN   33
#define DIR_PIN    32

// ── Зуммер ───────────────────────────
#define BUZZER_PIN 27


// ── Датчики света ─────────────────────
#define SENSOR_MOTOR 34    
#define SENSOR_STEPPER  35   

// ── PCA9685 (сервы) ──────────────────
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


// ── UART0 (USB, команды + отладка) ───
// TX=1, RX=3 — стандартный Serial, определён автоматически

#define MotorSerial  Serial1   // UART1 — моторы MG
#define MegaSerial   Serial2   // UART2 — Arduino Mega

// ── UART1 (Моторы MG, 3 шт) ──────────
#define UART1_TX  13
#define UART1_RX  14
#define UART1_BAUD 230400

// ── UART2 (Arduino Mega) ──────────────
#define UART2_TX  17
#define UART2_RX  16
#define UART2_BAUD 115200

// ── SoftwareSerial (вторая ESP32) ─────
#define SOFT_RX   4
#define SOFT_TX   23
#define SOFT_BAUD 9600
SoftwareSerial FlipperSerial(SOFT_RX, SOFT_TX);


// ── Позиции нот (мм) ────────────────
long x_note[7] = {18, 35, 51, 66, 80, 94, 113};

// ── Позиции пальцев (заполняются в init) ──
long fingers_x[4] = {0, 0, 0, 0};

// ── Шагов на мм ──────────────────────
int pulses_per_mm = 40;

// ── Углы серв: вверх / вниз ──────────
//    [0] Указательный
//    [1] Средний
//    [2] Безымянный
//    [3] Мизинец
int finger_up[4]   = {50,  50,  0,   0};
int finger_down[4] = {0,   0,   50,  50};


// ╔══════════════════════════════════════════════╗
// ║           ОБЪЕКТЫ                            ║
// ╚══════════════════════════════════════════════╝

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Спиннер ──────────────────────────
const char spinnerFrames[] = {'|', '/', '-', '\\'};
int spinnerIndex = 0;
unsigned long lastSpinner = 0;


// ╔══════════════════════════════════════════════╗
// ║           OLED                               ║
// ╚══════════════════════════════════════════════╝

void animateStartup() {
    display.clearDisplay();
    display.display();
    delay(100);

    // "EMMA" побуквенно
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    String top = "EMMA";
    for (int i = 0; i < (int)top.length(); i++) {
        display.setCursor(4 + i * 14, 0);
        display.print(top[i]);
        display.display();
        delay(120);
    }

    // Две линии
    display.drawFastHLine(0, 17, SCREEN_WIDTH, SSD1306_WHITE);
    display.display();
    delay(80);

    display.drawFastHLine(0, 22, SCREEN_WIDTH, SSD1306_WHITE);
    display.display();
    delay(80);

    // "MAIN CONTROLLER" побуквенно
    display.setTextSize(1);
    String bot = "MAIN CONTROLLER";
    for (int i = 0; i < (int)bot.length(); i++) {
        display.setCursor(4 + i * 7, 25);
        display.print(bot[i]);
        display.display();
        delay(60);
    }

    delay(150);

    // Мигание
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

    // EMMA слева
    display.setTextSize(2);
    display.setCursor(4, 0);
    display.print("EMMA");

    // Спиннер справа вверху
    display.setTextSize(1);
    display.setCursor(118, 0);
    display.print(spinnerFrames[spinnerIndex]);

    // Зона инфо (y=20..31) — сюда будем добавлять
    display.setTextSize(1);
    display.setCursor(4, 20);
    display.print("READY");

    display.display();
}


// ╔══════════════════════════════════════════════╗
// ║           СЕРВЫ                              ║
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



void calibrate() {
    Serial.println("[CAL] Отъезжаем от датчика...");
    stepper_set_target(400, 4000, 1000, 50, 50);
    while (!stepper_is_ready()) delay(1);

    Serial.println("[CAL] Едем к датчику...");
    stepper_set_target(-999999, 4000, 1000, 50, 50);

    while (digitalRead(SENSOR_STEPPER) == HIGH) delay(1);

    stepper_stop();
    stepper_reset();

    Serial.println("[CAL] Датчик! Отъезжаем...");

    stepper_set_target(2000, 4000, 1000, 50, 50);
    while (!stepper_is_ready()) delay(1);

    stepper_reset();
    Serial.println("[CAL] Готово");
}


//=================================================//
//=================================================//
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

String cmd()         { return _args[0]; }
int    getInt(int i) { return _args[i].toInt(); }



void handleUSB() {
    if (!Serial.available()) return;
    String input = Serial.readString();
    parse(input);

    showOnScreen(input);

    if (cmd() == "/set_note") {
        set_note(getInt(1), x_note, fingers_x);
    }
    if (cmd() == "/finger") {
        activate_finger(getInt(1));
    }
    if (cmd() == "/calibrate") {
        calibrate();
    }
}


//=================================================//
//=================================================//


// ╔══════════════════════════════════════════════╗
// ║           SETUP                              ║
// ╚══════════════════════════════════════════════╝

void setup() {
    
    FlipperSerial.begin(SOFT_BAUD);
    Serial.begin(115200);
    Serial.setTimeout(5);
    
    // UART1 — моторы MG
    MotorSerial.begin(UART1_BAUD, SERIAL_8N1, UART1_RX, UART1_TX);
    // UART2 — Arduino Mega
    MegaSerial.begin(UART2_BAUD, SERIAL_8N1, UART2_RX, UART2_TX);

    // Зуммер
    pinMode(BUZZER_PIN, OUTPUT);

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // LIGHT_SENSOR
    pinMode(SENSOR_MOTOR,   INPUT);
    pinMode(SENSOR_STEPPER, INPUT);

    // OLED
    display.setRotation(2);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.display();
    animateStartup();

    // PCA9685 (сервы)
    pwm.begin();
    pwm.setPWMFreq(PCA_FREQ);
    for (int i = 0; i < SERVO_COUNT; i++) {
        pwm.setPWM(i, 0, angle_to_pulse(finger_up[i]));
    }

    // Шаговый мотор
    stepper_begin(STEP_PIN, DIR_PIN);

    // Пальцы
    fingers_init(x_note, 7, fingers_x, pulses_per_mm);

    Serial.println("=== EMMA READY ===");


    calibrate();


}


// ╔══════════════════════════════════════════════╗
// ║           LOOP                               ║
// ╚══════════════════════════════════════════════╝

void loop() {

    handleUSB();
     fingers_update(fingers_x);
     
    // Спиннер на экране каждые 500мс
    if (millis() - lastSpinner > 500) {
        lastSpinner = millis();
        spinnerIndex = (spinnerIndex + 1) % 4;
        drawMainScreen();
    }
}

*/


/*
#include <Arduino.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SDA_PIN 33
#define SCL_PIN 32
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

BluetoothSerial SerialBT;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int msgCount = 0;

void showText(String line1, String line2 = "") {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(line1);
    display.setTextSize(1);
    display.setCursor(0, 40);
    display.print(line2);
    display.display();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);

    SerialBT.begin("EMMA");
    SerialBT.setPin("1234");

    showText("EMMA", "Waiting BT...");
    Serial.println("Ready");
}

void loop() {
    if (SerialBT.available()) {
        String msg = SerialBT.readStringUntil('\n');
        msg.trim();
        msgCount++;
        
        showText(msg, "#" + String(msgCount));
        Serial.println("BT: " + msg);
        
        // отправляем обратно подтверждение
        SerialBT.println("OK:" + msg);
    }
}
    */




/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define STATE_PIN  18
#define BUZZER_PIN 25

// MAC адрес второй ESP32
uint8_t peerAddress[] = {0x1C, 0xC3, 0xAB, 0xF9, 0x31, 0x5C};

// Структура данных для ESP-NOW
typedef struct {
    char cmd[32];
} EspNowMessage;

EspNowMessage outMsg;
esp_now_peer_info_t peerInfo;

String msg = "";
long highSince = 0;
bool isConnected = false;
bool flag_connect = false;

unsigned long try_bluetooth_startTime = 0;

String last_cmd = "";
bool displayDot = false;
unsigned long lastDisplayUpdate = 0;

// ── ESP-NOW callback отправки ───────────
void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    Serial.print("ESP-NOW send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ── Инициализация ESP-NOW ───────────────
void setupEspNow() {
    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }
    esp_now_register_send_cb(onSent);

    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if(esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("ESP-NOW add peer failed!");
        return;
    }
    Serial.println("ESP-NOW ready!");
}

// ── Отправка через ESP-NOW ──────────────
void sendEspNow(String cmd) {
    memset(outMsg.cmd, 0, sizeof(outMsg.cmd));
    cmd.toCharArray(outMsg.cmd, sizeof(outMsg.cmd));
    esp_now_send(peerAddress, (uint8_t*)&outMsg, sizeof(outMsg));
    Serial.println("ESP-NOW >> " + cmd);
}

// ── Анимация появления ─────────────────
void animateStartup() {
    for(int y = 0; y < SCREEN_HEIGHT; y += 2) {
        display.drawFastHLine(0, y, SCREEN_WIDTH, SSD1306_WHITE);
        display.display();
        delay(18);
    }
    delay(120);
    display.clearDisplay();
    display.display();
    delay(100);

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    String name = "EMMA";
    for(int i = 0; i < (int)name.length(); i++) {
        display.setCursor(4 + i * 14, 2);
        display.print(name[i]);
        display.display();
        delay(120);
    }

    display.drawFastVLine(63, 0, SCREEN_HEIGHT, SSD1306_WHITE);
    display.display();
    delay(80);

    display.setTextSize(1);
    display.setCursor(68, 4);  display.print("BT");      display.display(); delay(80);
    display.setCursor(66, 16); display.print("MODULE");   display.display(); delay(80);
    display.setCursor(4, 20);  display.print("PROJECT");  display.display(); delay(100);

    for(int i = 0; i < 3; i++) {
        display.invertDisplay(true);  delay(80);
        display.invertDisplay(false); delay(80);
    }
}

// ── Иконка блютуза ──────────────────────
void drawBTIcon(int x, int y) {
    display.drawFastVLine(x + 3, y, 10, SSD1306_WHITE);
    display.drawLine(x + 3, y,     x + 6, y + 3,  SSD1306_WHITE);
    display.drawLine(x + 6, y + 3, x + 3, y + 6,  SSD1306_WHITE);
    display.drawLine(x + 3, y + 6, x + 6, y + 9,  SSD1306_WHITE);
    display.drawLine(x + 6, y + 9, x + 3, y + 12, SSD1306_WHITE);
}

// ── Главный экран ──────────────────────
    void drawMainScreen(bool btDot, String cmd, bool connectedStatus) {
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 2);
    display.print("EMMA");

    display.setTextSize(1);
    display.drawFastVLine(63, 0, SCREEN_HEIGHT, SSD1306_WHITE);

    if(connectedStatus) {
        display.setCursor(4, 22);
        display.print("FLIPPER");
        display.setCursor(68, 4);
        display.print(cmd == "" ? "READY" : cmd);
    } else {
        display.setCursor(4, 22);
        display.print("PROJECT");
        drawBTIcon(66, 2);
        display.setCursor(78, 4);  display.print("BT");
        display.setCursor(66, 20); display.print("MODULE");
    }

    if(btDot) display.fillCircle(122, 28, 2, SSD1306_WHITE);

    display.display();
}

void try_bluetooth_reset_timer() {
    try_bluetooth_startTime = millis();
}

bool try_bluetooth_timer(int interval) {
    return millis() - try_bluetooth_startTime >= (unsigned long)interval;
}

void beep(int freq, int duration) {
    tone(BUZZER_PIN, freq, duration);
    delay(duration);
    noTone(BUZZER_PIN);
}

void soundConnected() {
    beep(1000, 100); delay(50);
    beep(1500, 100); delay(50);
    beep(2000, 150);
}

void soundDisconnected() {
    beep(2000, 100); delay(50);
    beep(1500, 100); delay(50);
    beep(1000, 200);
}

void sendAT(String cmd) {
    Serial2.println(cmd);
    Serial.println(">> " + cmd);
    delay(100);
    while(Serial2.available()) {
        Serial.print("<< ");
        Serial.println(Serial2.readString());
    }
}

void setupHM10() {
    sendAT("AT+ERASE");
}

void setupDataPath() {
    delay(300);
    sendAT("AT+SET_WAYWR0021");
    sendAT("AT+INDICA_ON0023");
    Serial.println("Data path ready!");
}

bool checkConnectedBlocking() {
    Serial.println("Checking connection for 1 sec...");
    long start = millis();
    int highCount = 0, samples = 0;
    while(millis() - start < 1000) {
        if(digitalRead(STATE_PIN) == HIGH) highCount++;
        samples++;
        delay(10);
    }
    float avg = (float)highCount / samples;
    Serial.print("PIN avg: "); Serial.println(avg);
    return avg > 0.8;
}

bool checkConnectionByPin() {
    if(digitalRead(STATE_PIN) == HIGH) {
        if(highSince == 0) highSince = millis();
        if(millis() - highSince > 1200) return true;
    } else {
        highSince = 0;
    }
    return false;
}

// ── Обработка команды ──────────────────
void handleCommand(String cmd) {
    Serial.println("CMD: " + cmd);
    last_cmd = cmd;
    sendEspNow(cmd);
    drawMainScreen(displayDot, last_cmd, isConnected);
    beep(2500, 30);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 17, 16);

    pinMode(STATE_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    delay(500);

    Wire.begin(33, 32);
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("OLED не найден!");
    } else {
        display.clearDisplay();
        display.display();
        animateStartup();
    }

    setupEspNow();

    beep(500, 100); delay(50); beep(1000, 100);

    if(checkConnectedBlocking()) {
        Serial.println("Already connected!");
        isConnected = true;
        setupDataPath();
        soundConnected();
        flag_connect = true;
    } else {
        Serial.println("Not connected. Setting up...");
        setupHM10();
        Serial.println("Waiting for Flipper...");
        while(!checkConnectionByPin()) { delay(10); }
        Serial.println("Connected!");
        isConnected = true;
        setupDataPath();
        soundConnected();
        flag_connect = true;
    }

    Serial.println("Ready!");
    try_bluetooth_reset_timer();
    lastDisplayUpdate = millis();
}

void loop() {
    bool connected = checkConnectionByPin();

    if(!connected) {
        if(try_bluetooth_timer(5000)) {
            Serial.println("DISCONNECTED");
            soundDisconnected();
            try_bluetooth_reset_timer();
            setupHM10();
            flag_connect = false;
            isConnected  = false;
            last_cmd     = "";
            drawMainScreen(displayDot, last_cmd, isConnected);
        }
    } else {
        try_bluetooth_reset_timer();
        if(!flag_connect) {
            flag_connect = true;
            isConnected  = true;
            setupDataPath();
            soundConnected();
            last_cmd = "";
            drawMainScreen(displayDot, last_cmd, isConnected);
        }
    }

    if(millis() - lastDisplayUpdate > 600) {
        displayDot = !displayDot;
        lastDisplayUpdate = millis();
        drawMainScreen(displayDot, last_cmd, isConnected);
    }

    if(Serial2.available()) {
        while(Serial2.available()) {
            msg += (char)Serial2.read();
        }

        Serial.print("RAW: ["); Serial.print(msg); Serial.println("]");

        if(isConnected) {
            if(msg.indexOf("DRIVE:FORWARD") >= 0)      handleCommand("FORWARD");
            else if(msg.indexOf("DRIVE:BACK")    >= 0) handleCommand("BACK");
            else if(msg.indexOf("DRIVE:LEFT")    >= 0) handleCommand("LEFT");
            else if(msg.indexOf("DRIVE:RIGHT")   >= 0) handleCommand("RIGHT");
            else if(msg.indexOf("DRIVE:STOP")    >= 0) handleCommand("STOP");
            else if(msg.indexOf("HAND:CALIB")    >= 0) handleCommand("H:CALIB");
            else if(msg.indexOf("HAND:")         >= 0) {
                int colon = msg.indexOf("HAND:") + 5;
                int semi  = msg.indexOf(";");
                if(semi > colon) {
                    String pos   = msg.substring(colon, semi);
                    String press = msg.indexOf("PRESS") >= 0 ? "P" : "R";
                    handleCommand("H:" + pos + ";" + press);
                }
            }
            else if(msg.indexOf("CALIB:ON")     >= 0) handleCommand("CAL:ON");
            else if(msg.indexOf("CALIB:OFF")    >= 0) handleCommand("CAL:OFF");
            else if(msg.indexOf("STARTPOS:ON")  >= 0) handleCommand("SP:ON");
            else if(msg.indexOf("STARTPOS:OFF") >= 0) handleCommand("SP:OFF");
            else if(msg.indexOf("SOUND:ON")     >= 0) handleCommand("SND:ON");
            else if(msg.indexOf("SOUND:OFF")    >= 0) handleCommand("SND:OFF");
        }

        msg = "";
    }
}

*/


/*



#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// ── OLED ────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Пины ────────────────────────────────
#define STATE_PIN  18
#define BUZZER_PIN 25

// ── ESP-NOW ─────────────────────────────
uint8_t peerAddress[] = {0x1C, 0xC3, 0xAB, 0xF9, 0x31, 0x5C};

typedef struct {
    char cmd[32];
} EspNowMessage;

EspNowMessage outMsg;
esp_now_peer_info_t peerInfo;

// ── Bluetooth Classic (Flutter) ─────────
BluetoothSerial SerialBT;
constexpr char kDeviceName[] = "ESP32 Command Bridge";
bool lastClientState = false;
String currentIncomingPayload;

// ── Состояние ───────────────────────────
String msg = "";
long highSince = 0;
bool isConnected = false;
bool flag_connect = false;

unsigned long try_bluetooth_startTime = 0;

String last_cmd = "";
bool displayDot = false;
unsigned long lastDisplayUpdate = 0;

// ── ESP-NOW callback отправки ───────────
void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    Serial.print("ESP-NOW send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ── Инициализация ESP-NOW ───────────────
void setupEspNow() {
    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        return;
    }
    esp_now_register_send_cb(onSent);

    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if(esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("ESP-NOW add peer failed!");
        return;
    }
    Serial.println("ESP-NOW ready!");
}

// ── Отправка через ESP-NOW ──────────────
void sendEspNow(String cmd) {
    memset(outMsg.cmd, 0, sizeof(outMsg.cmd));
    cmd.toCharArray(outMsg.cmd, sizeof(outMsg.cmd));
    esp_now_send(peerAddress, (uint8_t*)&outMsg, sizeof(outMsg));
    Serial.println("ESP-NOW >> " + cmd);
}

// ── Анимация появления ─────────────────
void animateStartup() {
    for(int y = 0; y < SCREEN_HEIGHT; y += 2) {
        display.drawFastHLine(0, y, SCREEN_WIDTH, SSD1306_WHITE);
        display.display();
        delay(18);
    }
    delay(120);
    display.clearDisplay();
    display.display();
    delay(100);

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    String name = "EMMA";
    for(int i = 0; i < (int)name.length(); i++) {
        display.setCursor(4 + i * 14, 2);
        display.print(name[i]);
        display.display();
        delay(120);
    }

    display.drawFastVLine(63, 0, SCREEN_HEIGHT, SSD1306_WHITE);
    display.display();
    delay(80);

    display.setTextSize(1);
    display.setCursor(68, 4);  display.print("BT");      display.display(); delay(80);
    display.setCursor(66, 16); display.print("MODULE");   display.display(); delay(80);
    display.setCursor(4, 20);  display.print("PROJECT");  display.display(); delay(100);

    for(int i = 0; i < 3; i++) {
        display.invertDisplay(true);  delay(80);
        display.invertDisplay(false); delay(80);
    }
}

// ── Иконка блютуза ──────────────────────
void drawBTIcon(int x, int y) {
    display.drawFastVLine(x + 3, y, 10, SSD1306_WHITE);
    display.drawLine(x + 3, y,     x + 6, y + 3,  SSD1306_WHITE);
    display.drawLine(x + 6, y + 3, x + 3, y + 6,  SSD1306_WHITE);
    display.drawLine(x + 3, y + 6, x + 6, y + 9,  SSD1306_WHITE);
    display.drawLine(x + 6, y + 9, x + 3, y + 12, SSD1306_WHITE);
}

// ── Главный экран ──────────────────────
void drawMainScreen(bool btDot, String cmd, bool connectedStatus) {
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 2);
    display.print("EMMA");

    display.setTextSize(1);
    display.drawFastVLine(63, 0, SCREEN_HEIGHT, SSD1306_WHITE);

    if(connectedStatus) {
        display.setCursor(4, 22);
        display.print("FLIPPER");
        display.setCursor(68, 4);
        display.print(cmd == "" ? "READY" : cmd);
    } else {
        display.setCursor(4, 22);
        display.print("PROJECT");
        drawBTIcon(66, 2);
        display.setCursor(78, 4);  display.print("BT");
        display.setCursor(66, 20); display.print("MODULE");
    }

    if(btDot) display.fillCircle(122, 28, 2, SSD1306_WHITE);

    display.display();
}

// ── Таймер ──────────────────────────────
void try_bluetooth_reset_timer() {
    try_bluetooth_startTime = millis();
}

bool try_bluetooth_timer(int interval) {
    return millis() - try_bluetooth_startTime >= (unsigned long)interval;
}

// ── Звуки ───────────────────────────────
void beep(int freq, int duration) {
    tone(BUZZER_PIN, freq, duration);
    delay(duration);
    noTone(BUZZER_PIN);
}

void soundConnected() {
    beep(1000, 100); delay(50);
    beep(1500, 100); delay(50);
    beep(2000, 150);
}

void soundDisconnected() {
    beep(2000, 100); delay(50);
    beep(1500, 100); delay(50);
    beep(1000, 200);
}

// ── HM-10 AT-команды ────────────────────
void sendAT(String cmd) {
    Serial2.println(cmd);
    Serial.println(">> " + cmd);
    delay(100);
    while(Serial2.available()) {
        Serial.print("<< ");
        Serial.println(Serial2.readString());
    }
}

void setupHM10() {
    sendAT("AT+ERASE");
}

void setupDataPath() {
    delay(300);
    sendAT("AT+SET_WAYWR0021");
    sendAT("AT+INDICA_ON0023");
    Serial.println("Data path ready!");
}

// ── Проверка подключения Flipper ────────
bool checkConnectedBlocking() {
    Serial.println("Checking connection for 1 sec...");
    long start = millis();
    int highCount = 0, samples = 0;
    while(millis() - start < 1000) {
        if(digitalRead(STATE_PIN) == HIGH) highCount++;
        samples++;
        delay(10);
    }
    float avg = (float)highCount / samples;
    Serial.print("PIN avg: "); Serial.println(avg);
    return avg > 0.8;
}

bool checkConnectionByPin() {
    if(digitalRead(STATE_PIN) == HIGH) {
        if(highSince == 0) highSince = millis();
        if(millis() - highSince > 1200) return true;
    } else {
        highSince = 0;
    }
    return false;
}

// ── Обработка команды (Flipper) ─────────
void handleCommand(String cmd) {
    Serial.println("CMD: " + cmd);
    last_cmd = cmd;
    sendEspNow(cmd);
    drawMainScreen(displayDot, last_cmd, isConnected);  
    beep(2500, 30);
}

// ── Обработка команды (Flutter) ─────────
void handleFlutterCommand(String cmd) {
    Serial.println("FLUTTER CMD: " + cmd);
    last_cmd = cmd;
    sendEspNow(cmd);
    drawMainScreen(displayDot, last_cmd, isConnected);
    beep(2500, 30);
}

// ── Приём символа из BT Classic ─────────
void storeIncomingChar(const char value) {
    if (value == '\r') {
        return;
    }

    if (value == '\0' || value == '\n') {
        if (currentIncomingPayload.length() > 0) {
            Serial.print("BT Classic received: ");
            Serial.println(currentIncomingPayload);
            handleFlutterCommand(currentIncomingPayload);
            currentIncomingPayload = "";
        }
        return;
    }

    currentIncomingPayload += value;
}

// ═════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 17, 16);

    pinMode(STATE_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    delay(500);

    // OLED
    Wire.begin(33, 32);
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("OLED не найден!");
    } else {
        display.clearDisplay();
        display.display();
        animateStartup();
    }

    // ESP-NOW
    setupEspNow();

    // Bluetooth Classic (Flutter)
    if (!SerialBT.begin(kDeviceName)) {
        Serial.println("BT Classic init failed!");
    } else {
        Serial.print("BT Classic ready: ");
        Serial.println(kDeviceName);
    }

    beep(500, 100); delay(50); beep(1000, 100);

    // Проверка подключения Flipper (HM-10)
    if(checkConnectedBlocking()) {
        Serial.println("Already connected!");
        isConnected = true;
        setupDataPath();
        soundConnected();
        flag_connect = true;
    } else {
        Serial.println("Not connected. Setting up...");
        setupHM10();
        Serial.println("Waiting for Flipper...");
        while(!checkConnectionByPin()) { delay(10); }
        Serial.println("Connected!");
        isConnected = true;
        setupDataPath();
        soundConnected();
        flag_connect = true;
    }

    Serial.println("Ready!");
    try_bluetooth_reset_timer();
    lastDisplayUpdate = millis();
}

// ═════════════════════════════════════════
void loop() {
    // ── Flipper connection check ────────
    bool connected = checkConnectionByPin();

    if(!connected) {
        if(try_bluetooth_timer(5000)) {
            Serial.println("DISCONNECTED");
            soundDisconnected();
            try_bluetooth_reset_timer();
            setupHM10();
            flag_connect = false;
            isConnected  = false;
            last_cmd     = "";
            drawMainScreen(displayDot, last_cmd, isConnected);
        }
    } else {
        try_bluetooth_reset_timer();
        if(!flag_connect) {
            flag_connect = true;
            isConnected  = true;
            setupDataPath();
            soundConnected();
            last_cmd = "";
            drawMainScreen(displayDot, last_cmd, isConnected);
        }
    }

    // ── Обновление экрана (мигающая точка) ──
    if(millis() - lastDisplayUpdate > 600) {
        displayDot = !displayDot;
        lastDisplayUpdate = millis();
        drawMainScreen(displayDot, last_cmd, isConnected);
    }

    // ── Приём данных от Flipper (HM-10 Serial2) ──
    if(Serial2.available()) {
        while(Serial2.available()) {
            msg += (char)Serial2.read();
        }

        Serial.print("RAW: ["); Serial.print(msg); Serial.println("]");

        if(isConnected) {
            if(msg.indexOf("DRIVE:FORWARD") >= 0)      handleCommand("FORWARD");
            else if(msg.indexOf("DRIVE:BACK")    >= 0) handleCommand("BACK");
            else if(msg.indexOf("DRIVE:LEFT")    >= 0) handleCommand("LEFT");
            else if(msg.indexOf("DRIVE:RIGHT")   >= 0) handleCommand("RIGHT");
            else if(msg.indexOf("DRIVE:STOP")    >= 0) handleCommand("STOP");
            else if(msg.indexOf("HAND:CALIB")    >= 0) handleCommand("H:CALIB");
            else if(msg.indexOf("HAND:")         >= 0) {
                int colon = msg.indexOf("HAND:") + 5;
                int semi  = msg.indexOf(";");
                if(semi > colon) {
                    String pos   = msg.substring(colon, semi);
                    String press = msg.indexOf("PRESS") >= 0 ? "P" : "R";
                    handleCommand("H:" + pos + ";" + press);
                }
            }
            else if(msg.indexOf("CALIB:ON")     >= 0) handleCommand("CAL:ON");
            else if(msg.indexOf("CALIB:OFF")    >= 0) handleCommand("CAL:OFF");
            else if(msg.indexOf("STARTPOS:ON")  >= 0) handleCommand("SP:ON");
            else if(msg.indexOf("STARTPOS:OFF") >= 0) handleCommand("SP:OFF");
            else if(msg.indexOf("SOUND:ON")     >= 0) handleCommand("SND:ON");
            else if(msg.indexOf("SOUND:OFF")    >= 0) handleCommand("SND:OFF");
        }

        msg = "";
    }

    // ── Приём данных от Flutter (BT Classic) ──
    bool currentClientState = SerialBT.hasClient();
    if (currentClientState != lastClientState) {
        if (currentClientState) {
            Serial.println("Flutter client connected");
        } else {
            Serial.println("Flutter client disconnected");
        }
        lastClientState = currentClientState;
    }

    while (SerialBT.available()) {
        char c = SerialBT.read();
        storeIncomingChar(c);
    }
}


*/



#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t peerMAC[] = {0x70, 0x4B, 0xCA, 0x4E, 0x27, 0xD8};

String lastReply = "";
bool delivered = false;
int sentCount = 0;

void showScreen(String line1, String line2 = "") {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(line1);
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.print(line2);
    display.display();
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    delivered = (status == ESP_NOW_SEND_SUCCESS);
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    char buf[251];
    int copyLen = len < 250 ? len : 250;
    memcpy(buf, data, copyLen);
    buf[copyLen] = '\0';
    lastReply = String(buf);
    Serial.print("<< Reply: ");
    Serial.println(lastReply);
    showScreen("REPLY", lastReply);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);

    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, peerMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    showScreen("MAIN", "Ready");
    Serial.println("MAIN ready. Type commands.");
}

void loop() {
    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        sentCount++;

        esp_now_send(peerMAC, (uint8_t*)msg.c_str(), msg.length());
        showScreen("SENT #" + String(sentCount), msg);
        Serial.print(">> Sent: ");
        Serial.println(msg);
    }
}