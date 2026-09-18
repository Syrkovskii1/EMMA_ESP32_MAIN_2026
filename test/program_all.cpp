/*
#include <Arduino.h>
#include "EMMA_Stepper_Motor.h"

#define STEP_PIN 25
#define DIR_PIN  26

long STEPS = 1600;
long SPEED = 20000;

unsigned long move_start_ms = 0;
bool was_ready = true;

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5);
    stepper_begin(STEP_PIN, DIR_PIN);

    Serial.println("=== ТЕСТ ===");
    Serial.println("/spd;8000   - скорость");
    Serial.println("/steps;1600 - шагов");
    Serial.println("/stop       - стоп");
    Serial.println("/pos        - позиция");
    Serial.println("============");

    move_start_ms = millis();
    stepper_set_target(STEPS, SPEED);
}

void loop() {
    if (Serial.available() > 0) {
        String s = Serial.readString();
        s.trim();
        int sep    = s.indexOf(';');
        String cmd = s.substring(0, sep);
        long   arg = s.substring(sep + 1).toInt();

        if (cmd == "/spd") {
            SPEED = arg;
            Serial.print("Скорость: "); Serial.println(SPEED);
        }
        else if (cmd == "/steps") {
            STEPS = arg;
            Serial.print("Шагов: "); Serial.println(STEPS);
        }
        else if (cmd == "/stop") {
            stepper_stop();
            Serial.println("Стоп");
        }
        else if (cmd == "/pos") {
            Serial.print("Позиция: "); Serial.println(stepper_get_current());
        }
    }

    // Фиксируем момент старта движения
    if (was_ready && !stepper_is_ready()) {
        move_start_ms = millis();
        was_ready = false;
    }

    if (stepper_is_ready() && !was_ready) {
        was_ready = true;

        unsigned long move_time = millis() - move_start_ms;
        long cur = stepper_get_current();

        Serial.print("Позиция: ");   Serial.println(cur);
        Serial.print("Время мс: ");  Serial.println(move_time);
        Serial.println("---");

        delay(50);

        move_start_ms = millis();
        if (cur == 0) {
            stepper_set_target(STEPS, SPEED);
        } else {
            stepper_set_target(0, SPEED);
        }
    }
}
*/


/*
#include <Arduino.h>
#include "EMMA_Stepper_Motor.h"
#include "EMMA_Fingers.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);


// Каналы на шилде
#define CH_INDEX   0
#define CH_MIDDLE  1
#define CH_RING    2
#define CH_PINKIE  3

// Углы → импульсы PCA9685 (150-600 примерно 0-180 градусов)
int angle_to_pulse(int angle) {
    return map(angle, 0, 180, 150, 600);
}

// Углы поднят / опущен для каждого пальца
int finger_up[4]   = {50,  50,  0,   0};   // поднят
int finger_down[4] = {0,   0,   50,  50};  // опущен


#define STEP_PIN 25
#define DIR_PIN  26

long x_note[7]     = {18, 35, 51, 66, 80, 94, 113};
long fingers_x[4]  = {0, 0, 0, 0};
int  pulses_per_mm = 40;



void activate_finger(int index) {
    // Все пальцы вверх
    for (int i = 0; i < 4; i++) {
        pwm.setPWM(i, 0, angle_to_pulse(finger_up[i]));
    }
    // Нужный палец вниз
    if (index >= 0 && index < 4) {
        pwm.setPWM(index, 0, angle_to_pulse(finger_down[index]));
    }
}


void print_fingers(long* notes, long* fingers) {
    Serial.println("--- Ноты (шаги) ---");
    for (int i = 0; i < 7; i++) {
        Serial.print("  Нота "); Serial.print(i + 1);
        Serial.print(": ");     Serial.println(notes[i]);
    }
    Serial.println("--- Пальцы (шаги) ---");
    Serial.print("  [0] Указательный: "); Serial.println(fingers[0]);
    Serial.print("  [1] Средний:      "); Serial.println(fingers[1]);
    Serial.print("  [2] Безымянный:   "); Serial.println(fingers[2]);
    Serial.print("  [3] Мизинец:      "); Serial.println(fingers[3]);
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5);

    Wire.begin(21, 22);
    pwm.begin();
    pwm.setPWMFreq(50); // 50Hz для серв



    stepper_begin(STEP_PIN, DIR_PIN);
    
    fingers_init(x_note, 7, fingers_x, pulses_per_mm);

    Serial.println("=== ТЕСТ ПАЛЬЦЕВ ===");
    Serial.println("/note;1..7  - выбрать ноту");
    Serial.println("/pos        - позиция каретки");
    Serial.println("/fingers    - позиции пальцев");
    Serial.println("/stop       - стоп");
    Serial.println("===================");

    print_fingers(x_note, fingers_x);
}



void loop() {
    // Обновляем логику пальцев
    fingers_update(fingers_x);

    if (Serial.available() > 0) {
        String s = Serial.readString();
        s.trim();
        int sep    = s.indexOf(';');
        String cmd = s.substring(0, sep);
        long   arg = s.substring(sep + 1).toInt();

        if (cmd == "/note") {
            int nota = (int)arg;
            if (nota < 1 || nota > 7) {
                Serial.println("[ERR] Нота 1..7");
                return;
            }

            // Показываем что будет ДО движения
            long target = x_note[nota - 1];
            int  finger = find_nearest_finger(target, fingers_x);
            long delta  = target - fingers_x[finger];

            Serial.println("--- Расчёт ---");
            Serial.print("Нота:          "); Serial.println(nota);
            Serial.print("Цель (шаги):   "); Serial.println(target);
            Serial.print("Палец:         "); Serial.println(finger);
            Serial.print("Позиция пальца:"); Serial.println(fingers_x[finger]);
            Serial.print("Дельта:        "); Serial.println(delta);
            Serial.print("Каретка едет → "); 
            Serial.println(stepper_get_current() + delta);


            set_note(nota, x_note, fingers_x);
        }

        else if (cmd == "/pos") {
            Serial.print("Каретка: ");
            Serial.println(stepper_get_current());
        }

        else if (cmd == "/fingers") {
            print_fingers(x_note, fingers_x);
        }

        else if (cmd == "/stop") {
            stepper_stop();
            Serial.println("Стоп");
        }
    }
}
    */



/*
#include <Arduino.h>
#include "EMMA_Stepper_Motor.h"
#include "EMMA_Fingers.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ── Пины ─────────────────────────────
#define STEP_PIN 25
#define DIR_PIN  26

// ── Данные нот и пальцев ─────────────
long x_note[7]     = {18, 35, 51, 66, 80, 94, 113};
long fingers_x[4]  = {0, 0, 0, 0};
int  pulses_per_mm = 40;

// ── Углы серв ────────────────────────
int finger_up[4]   = {50,  50,  0,   0};
int finger_down[4] = {0,   0,   50,  50};

// ── Play режим ───────────────────────
bool playing = false;

bool play_press = true;

int  play_note = 1;
unsigned long play_timer = 0;
int play_pause = 1000;

// ── Функции ──────────────────────────
int angle_to_pulse(int angle) {
    return map(angle, 0, 180, 150, 600);
}

void activate_finger(int index) {
    for (int i = 0; i < 4; i++) {
        pwm.setPWM(i, 0, angle_to_pulse(finger_up[i]));
    }
    if (index >= 0 && index < 4) {
        pwm.setPWM(index, 0, angle_to_pulse(finger_down[index]));
    }
}

void print_fingers(long* notes, long* fingers) {
    Serial.println("--- Ноты (шаги) ---");
    for (int i = 0; i < 7; i++) {
        Serial.print("  Нота "); Serial.print(i + 1);
        Serial.print(": ");     Serial.println(notes[i]);
    }
    Serial.println("--- Пальцы (шаги) ---");
    Serial.print("  [0] Указательный: "); Serial.println(fingers[0]);
    Serial.print("  [1] Средний:      "); Serial.println(fingers[1]);
    Serial.print("  [2] Безымянный:   "); Serial.println(fingers[2]);
    Serial.print("  [3] Мизинец:      "); Serial.println(fingers[3]);
}

// ── Парсер ───────────────────────────
// Разбивает строку по ";" на 3 части
void parse_args(String s, String &cmd, long &arg1, long &arg2, bool &has_arg2) {
    int sep1 = s.indexOf(';');
    has_arg2 = false;
    arg1 = 0;
    arg2 = 0;

    if (sep1 < 0) {
        cmd = s;
        return;
    }

    cmd = s.substring(0, sep1);
    String rest = s.substring(sep1 + 1);

    int sep2 = rest.indexOf(';');
    if (sep2 < 0) {
        arg1 = rest.toInt();
    } else {
        arg1 = rest.substring(0, sep2).toInt();
        arg2 = rest.substring(sep2 + 1).toInt();
        has_arg2 = true;
    }
}

// ── Setup ────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5);

    Wire.begin(21, 22);
    pwm.begin();
    pwm.setPWMFreq(50);

    for (int i = 0; i < 4; i++) {
        pwm.setPWM(i, 0, angle_to_pulse(finger_up[i]));
    }

    stepper_begin(STEP_PIN, DIR_PIN);
    fingers_init(x_note, 7, fingers_x, pulses_per_mm);

    Serial.println("=== EMMA ===");
    Serial.println("/note;3;1     - нота 3, зажать");
    Serial.println("/note;3;0     - нота 3, не зажимать");
    Serial.println("/note;3       - нота 3, зажать (по умолчанию)");
    Serial.println("/play;1       - перебор с зажатием");
    Serial.println("/play;0       - перебор без зажатия");
    Serial.println("/play         - перебор с зажатием");
    Serial.println("/pause;500    - пауза мс");
    Serial.println("/pos          - позиция каретки");
    Serial.println("/fingers      - позиции пальцев");
    Serial.println("/stop         - стоп");
    Serial.println("============");

    print_fingers(x_note, fingers_x);
}

// ── Loop ─────────────────────────────
void loop() {
    fingers_update(fingers_x);

    if (Serial.available() > 0) {
        String s = Serial.readString();
        s.trim();

        String cmd;
        long arg1, arg2;
        bool has_arg2;
        parse_args(s, cmd, arg1, arg2, has_arg2);

        // ── /note;N;0/1 ──────────────
        if (cmd == "/note") {
            int nota = (int)arg1;
            if (nota < 1 || nota > 7) {
                Serial.println("[ERR] Нота 1..7");
                return;
            }
            playing = false;

            bool press = true;
            if (has_arg2) press = (arg2 != 0);

            long target = x_note[nota - 1];
            int  finger = find_nearest_finger(target, fingers_x);
            long delta  = target - fingers_x[finger];

            Serial.println("--- Расчёт ---");
            Serial.print("Нота:    "); Serial.println(nota);
            Serial.print("Палец:   "); Serial.println(finger);
            Serial.print("Дельта:  "); Serial.println(delta);
            Serial.print("Зажатие: "); Serial.println(press ? "ДА" : "НЕТ");

            set_note(nota, x_note, fingers_x, press);
        }

        // ── /play;0/1 ────────────────
        else if (cmd == "/play") {
            playing = true;
            play_press = (arg1 != 0);
            if (!has_arg2 && arg1 == 0) {
                // /play без аргументов — зажимать по умолчанию
                // но /play;0 — без зажатия
                // нужно отличить /play от /play;0
            }
            play_note = 1;
            play_timer = millis();
            Serial.print("[PLAY] Старт, зажатие: ");
            Serial.println(play_press ? "ДА" : "НЕТ");
        }

        // ── /pause;мс ────────────────
        else if (cmd == "/pause") {
            play_pause = (int)arg1;
            Serial.print("Пауза: "); Serial.println(play_pause);
        }

        // ── /pos ─────────────────────
        else if (cmd == "/pos") {
            Serial.print("Каретка: ");
            Serial.println(stepper_get_current());
        }

        // ── /fingers ─────────────────
        else if (cmd == "/fingers") {
            print_fingers(x_note, fingers_x);
        }

        // ── /stop ────────────────────
        else if (cmd == "/stop") {
            stepper_stop();
            playing = false;
            Serial.println("Стоп");
        }
    }

    // ── Автоматический перебор нот ───
    if (playing && stepper_is_ready()) {
        if (millis() - play_timer > play_pause) {
            Serial.print("[PLAY] Нота: "); Serial.print(play_note);
            Serial.print(" Зажатие: "); Serial.println(play_press ? "ДА" : "НЕТ");

            set_note(play_note, x_note, fingers_x, play_press);

            play_note++;
            if (play_note > 7) play_note = 1;

            play_timer = millis();
        }
    }
}
    */




//////========================================================================================================
//////========================================================================================================
//////========================================================================================================

/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Настройки экрана ---
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C


// --- Настройки нового UART (Serial1) ---
#define SERIAL1_RX 18  // Выбери свободный пин RX
#define SERIAL1_TX 19  // Выбери свободный пин TX


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Настройки основной программы ---
#define STATE_PIN  4
#define BUZZER_PIN 25

String msg = "";
long highSince = 0;
bool isConnected = false;
bool flag_connect = false; // Было ':' вместо ';'

int try_counter = 0; 

unsigned long try_bluetooth_startTime = 0;

// Переменные для экрана
String last_cmd = "";               // Последняя полученная команда
bool displayDot = false;            // Состояние мигающей точки экрана
unsigned long lastDisplayUpdate = 0;// Таймер обновления экрана

// ── Анимация появления (оставляем как было) ─────────────────
void animateStartup() {
    // 1. Экран заполняется полосами сверху вниз
    for (int y = 0; y < SCREEN_HEIGHT; y += 2) {
        display.drawFastHLine(0, y, SCREEN_WIDTH, SSD1306_WHITE);
        display.display();
        delay(18);
    }
    delay(120);

    // 2. Экран гаснет
    display.clearDisplay();
    display.display();
    delay(100);

    // 3. EMMA появляется побуквенно
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    String name = "EMMA";
    for (int i = 0; i < name.length(); i++) {
        display.setCursor(4 + i * 14, 2);
        display.print(name[i]);
        display.display();
        delay(120);
    }

    // 4. Разделитель появляется
    display.drawFastVLine(63, 0, SCREEN_HEIGHT, SSD1306_WHITE);
    display.display();
    delay(80);

    // 5. Правая часть появляется
    display.setTextSize(1);
    display.setCursor(68, 4);
    display.print("BT");
    display.display();
    delay(80);
    display.setCursor(66, 16);
    display.print("MODULE");
    display.display();
    delay(80);

    // 6. PROJECT появляется снизу
    display.setCursor(4, 20);
    display.print("PROJECT");
    display.display();
    delay(100);

    // 7. Мигание всего экрана 3 раза — готово
    for (int i = 0; i < 3; i++) {
        display.invertDisplay(true);
        delay(80);
        display.invertDisplay(false);
        delay(80);
    }
}

// ── Иконка блютуза (8x12 пикселей) ──────────────────────
void drawBTIcon(int x, int y) {
    display.drawFastVLine(x + 3, y, 10, SSD1306_WHITE);
    display.drawLine(x + 3, y,     x + 6, y + 3, SSD1306_WHITE);
    display.drawLine(x + 6, y + 3, x + 3, y + 6, SSD1306_WHITE);
    display.drawLine(x + 3, y + 6, x + 6, y + 9, SSD1306_WHITE);
    display.drawLine(x + 6, y + 9, x + 3, y + 12, SSD1306_WHITE);
}

// ── Динамический экран (меняется от статуса подключения) ─
void drawMainScreen(bool btDot, String cmd, bool connectedStatus) {
    display.clearDisplay();

    // Левая часть (верх всегда EMMA)
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 2);
    display.print("EMMA");

    display.setTextSize(1);
    // Центральный разделитель
    display.drawFastVLine(63, 0, SCREEN_HEIGHT, SSD1306_WHITE);

    if (connectedStatus) {
        // --- СОСТОЯНИЕ ПОДКЛЮЧЕНО ---
        display.setCursor(4, 22);
        display.print("FLIPPER"); // Подпись слева внизу

        // Вывод команды справа
        display.setCursor(68, 12); 
        if (cmd == "") {
            display.print("READY");
        } else {
            display.print(cmd); 
        }
    } else {
        // --- СОСТОЯНИЕ ОТКЛЮЧЕНО (Главный экран) ---
        display.setCursor(4, 22);
        display.print("PROJECT"); // Подпись слева внизу

        // Правая часть — блютуз
        drawBTIcon(66, 2);
        display.setCursor(78, 4);
        display.print("BT");
        display.setCursor(66, 20);
        display.print("MODULE");
    }

    // Мигающая точка — индикатор активности (работает всегда)
    if (btDot) {
        display.fillCircle(122, 28, 2, SSD1306_WHITE);
    }

    display.display();
}

// --- Функции основной программы ---

// Исправлено: добавлен тип данных int для интервала и исправлено имя переменной внутри
void try_bluetooth_reset_timer() {   
    try_bluetooth_startTime = millis();
}

// Исправлено: добавлен аргумент интервала в саму функцию
bool try_bluetooth_timer(int try_bluetooth_interval) {
    if (millis() - try_bluetooth_startTime >= (unsigned long)try_bluetooth_interval) {
        return true;
    }
    return false;
}

void beep(int freq, int duration) {
    tone(BUZZER_PIN, freq, duration);
    delay(duration);
    noTone(BUZZER_PIN);
}

void soundConnected() {
    beep(1000, 100);
    delay(50);
    beep(1500, 100);
    delay(50);
    beep(2000, 150);
}

void soundDisconnected() {
    beep(2000, 100);
    delay(50);
    beep(1500, 100);
    delay(50);
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
   // sendAT("AT+ERASE");
   // delay(100);
   // sendAT("AT+MODE2");
   // sendAT("AT+ROLE1");
   // sendAT("AT+IMME0");
   // sendAT("AT+TYPE3");
   // sendAT("AT+PASS280508");
   // sendAT("AT+NOTI1");
   // sendAT("AT+RESET");
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
    int highCount = 0;
    int samples = 0;

    while(millis() - start < 1000) {
        if(digitalRead(STATE_PIN) == HIGH) highCount++;
        samples++;
        delay(10);
    }

    float avg = (float)highCount / samples;
    Serial.print("PIN avg: ");
    Serial.println(avg);
    return avg > 0.7;
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

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    pinMode(STATE_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    delay(500);

    // --- Инициализация экрана ---
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("OLED не найден!");
    } else {
        display.clearDisplay();
        display.display();
        animateStartup(); // Показываем заставку при запуске
    }

    beep(500, 100);
    delay(50);
    beep(1000, 100);

    if(checkConnectedBlocking()) {
        Serial.println("Already connected! Skipping setup.");
        isConnected = true;
        setupDataPath();
        soundConnected();
        flag_connect = true;
    } else {
        Serial.println("Not connected. Setting up HM-10...");
        setupHM10();

        Serial.println("Waiting for Flipper...");
        while(!checkConnectionByPin()) {
            delay(10); // Небольшая задержка для стабильности
        }
        Serial.println("Connected!");
        isConnected = true;
        setupDataPath();
        soundConnected();
        flag_connect = true;
    }

    Serial.println("Ready!");
    try_bluetooth_reset_timer(); // Инициализация таймера перед loop
    lastDisplayUpdate = millis();
}

void loop() {
    bool connected = checkConnectionByPin();

    if(connected == false) {
        if(try_bluetooth_timer(5000)) {
            Serial.println("DISCONNECTED");
            soundDisconnected();
            try_bluetooth_reset_timer();
            setupHM10();
            flag_connect = false;
            isConnected = false; // Важно сбросить этот флаг
            last_cmd = "";       // Сбрасываем команду
            drawMainScreen(displayDot, last_cmd, isConnected); // Возвращаем стартовый экран
        }
    } else {
        // Если подключен, обновляем таймер, чтобы он не сработал сразу после обрыва
        try_bluetooth_reset_timer();
        
        if (flag_connect == false) {
            flag_connect = true;
            isConnected = true;
            setupDataPath();
            soundConnected();
            last_cmd = ""; // Сбрасываем команду при новом подключении
            drawMainScreen(displayDot, last_cmd, isConnected);
        }
    }

    // --- Анимация точки на экране (каждые 600 мс) ---
    if (millis() - lastDisplayUpdate > 600) {
        displayDot = !displayDot;
        lastDisplayUpdate = millis();
        drawMainScreen(displayDot, last_cmd, isConnected); // Передаем isConnected для выбора фона
    }

    // --- Обработка Bluetooth ---
    if(Serial2.available()) {
        beep(2500, 30);
        
        while(Serial2.available()) {
            msg += (char)Serial2.read();
        }
        Serial.print("RAW: [");
        Serial.print(msg);
        Serial.println("]");

        if(isConnected) {
            bool commandReceived = false;

            if(msg.indexOf("FORWARD") >= 0) { Serial.println("FORWARD"); last_cmd = "FORWARD"; commandReceived = true; }
            if(msg.indexOf("STOP") >= 0)    { Serial.println("STOP");    last_cmd = "STOP";    commandReceived = true; }
            if(msg.indexOf("LEFT") >= 0)    { Serial.println("LEFT");    last_cmd = "LEFT";    commandReceived = true; }
            if(msg.indexOf("RIGHT") >= 0)   { Serial.println("RIGHT");   last_cmd = "RIGHT";   commandReceived = true; }
            if(msg.indexOf("BACK") >= 0)    { Serial.println("BACK");    last_cmd = "BACK";    commandReceived = true; }

            // Если пришла команда, сразу обновляем экран (с фоном "FLIPPER")
            if (commandReceived) {
                drawMainScreen(displayDot, last_cmd, isConnected);
                Serial1.println(last_cmd);
            }
        }
    }
    msg = "";
}
*/


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Spinner ---
const char spinnerFrames[] = {'|', '/', '-', '\\'};
int spinnerIndex = 0;
unsigned long lastSpinner = 0;

void animateStartup() {
    display.clearDisplay();
    display.display();
    delay(100);

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    String top = "EMMA";
    for (int i = 0; i < (int)top.length(); i++) {
        display.setCursor(4 + i * 14, 0);
        display.print(top[i]);
        display.display();
        delay(120);
    }

    display.drawFastHLine(0, 17, SCREEN_WIDTH, SSD1306_WHITE);
    display.display();
    delay(80);

    display.drawFastHLine(0, 22, SCREEN_WIDTH, SSD1306_WHITE);
    display.display();
    delay(80);

    display.setTextSize(1);
    String bot = "MAIN CONTROLLER";
    for (int i = 0; i < (int)bot.length(); i++) {
        display.setCursor(4 + i * 7, 25);
        display.print(bot[i]);
        display.display();
        delay(60);
    }

    delay(150);

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

    // Spinner правый верхний угол
    display.setTextSize(1);
    display.setCursor(118, 0);
    display.print(spinnerFrames[spinnerIndex]);

    // --- Зона инфо (y=18..31) ---
    // сюда потом пишешь свои переменные
    display.setTextSize(1);
    display.setCursor(4, 20);
    display.print("VAR1: ---");

    display.display();
}

void setup() {
    Wire.begin();
    display.setRotation(2); // 180 градусов
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.display();
    animateStartup();
}

void loop() {
    // Spinner каждые 250мс
    if (millis() - lastSpinner > 500) {
        lastSpinner = millis();
        spinnerIndex = (spinnerIndex + 1) % 4;
        drawMainScreen();
    }
}
    

/*
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define STATE_PIN  18
#define BUZZER_PIN 25

// Serial2 = HM-10 (RX=17, TX=16)
// Serial1 = вторая ESP32 (TX=22, RX=23)

String msg = "";
long highSince = 0;
bool isConnected = false;
bool flag_connect = false;

unsigned long try_bluetooth_startTime = 0;

String last_cmd = "";
bool displayDot = false;
unsigned long lastDisplayUpdate = 0;

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
    display.setCursor(68, 4);  display.print("BT");     display.display(); delay(80);
    display.setCursor(66, 16); display.print("MODULE");  display.display(); delay(80);
    display.setCursor(4, 20);  display.print("PROJECT"); display.display(); delay(100);

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
        if(cmd == "") {
            display.print("READY");
        } else {
            // Показываем команду крупнее
            display.setTextSize(1);
            display.setCursor(68, 4);
            display.print(cmd);
        }

    } else {
        display.setCursor(4, 22);
        display.print("PROJECT");

        drawBTIcon(66, 2);
        display.setCursor(78, 4);  display.print("BT");
        display.setCursor(66, 20); display.print("MODULE");
    }

    if(btDot) {
        display.fillCircle(122, 28, 2, SSD1306_WHITE);
    }

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

    // Отправляем на вторую ESP32
    Serial1.println(cmd);

    // Показываем на экране
    drawMainScreen(displayDot, last_cmd, isConnected);

    // Бипаем
    beep(2500, 30);
}

void setup() {
    Serial.begin(115200);
    

    // HM-10
    Serial2.begin(115200, SERIAL_8N1, 17, 16);

    // Вторая ESP32
    
    Serial1.begin(9600, SERIAL_8N1, 23, 22);
    Serial1.setTimeout(10);

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

    // Следим за состоянием подключения
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

    // Мигание точки
    if(millis() - lastDisplayUpdate > 600) {
        displayDot = !displayDot;
        lastDisplayUpdate = millis();
        drawMainScreen(displayDot, last_cmd, isConnected);
    }

    // Читаем данные от HM-10
    if(Serial2.available()) {
        while(Serial2.available()) {
            msg += (char)Serial2.read();
        }

        Serial.print("RAW: [");
        Serial.print(msg);
        Serial.println("]");

        if(isConnected) {
            // DRIVE команды
            if(msg.indexOf("DRIVE:FORWARD") >= 0) handleCommand("FORWARD");
            else if(msg.indexOf("DRIVE:BACK")    >= 0) handleCommand("BACK");
            else if(msg.indexOf("DRIVE:LEFT")    >= 0) handleCommand("LEFT");
            else if(msg.indexOf("DRIVE:RIGHT")   >= 0) handleCommand("RIGHT");
            else if(msg.indexOf("DRIVE:STOP")    >= 0) handleCommand("STOP");

            // HAND команды
            else if(msg.indexOf("HAND:CALIB")   >= 0) handleCommand("H:CALIB");
            else if(msg.indexOf("HAND:")         >= 0) {
                // Парсим HAND:3;PRESS или HAND:3;RELEASE
                int colon = msg.indexOf("HAND:") + 5;
                int semi  = msg.indexOf(";");
                if(semi > colon) {
                    String pos   = msg.substring(colon, semi);
                    String press = msg.indexOf("PRESS") >= 0 ? "P" : "R";
                    handleCommand("H:" + pos + ";" + press);
                }
            }

            // Остальные команды
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
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

#define SOFT_RX 4
#define SOFT_TX 23

SoftwareSerial softSerial(SOFT_RX, SOFT_TX);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

String lastCmd = "---";

void setup() {
    Serial.begin(115200);
    softSerial.begin(9600);
    softSerial.setTimeout(10);

    Wire.begin(21, 22);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setRotation(2);

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("WAITING");
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.println("SoftSerial 9600");
    display.setCursor(0, 45);
    display.println("RX:4  TX:23");
    display.display();

    Serial.println("=== SOFT SERIAL ===");
    Serial.println("Жду команды на GPIO 4...");
}

void loop() {
    if (softSerial.available()) {
        String s = softSerial.readString();
        s.trim();

        lastCmd = s;

        Serial.print("[SOFT] ");
        Serial.println(s);

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("RECEIVED:");
        display.setTextSize(2);
        display.setCursor(0, 20);
        display.println(lastCmd);
        display.display();
    }
}



*/

