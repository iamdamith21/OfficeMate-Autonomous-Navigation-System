/*
 * robot_firmware.ino v3 — OfficeMate sensor/actuator hub (4WD chassis)
 *
 * Board : Arduino Mega 2560
 * Libs  : Servo, LiquidCrystal I2C (Frank de Brabander), MFRC522 (GithubCommunity)
 *         (Wire + SPI ship with the core.)
 *
 * Drives the motors from /cmd_vel and exposes every other sensor/actuator
 * to the Pi over one USB-serial link. The ROS-side peer is
 * robot_interface/arduino_bridge. Odometry fusion (rf2o + this IMU) happens
 * on the Pi; this board never publishes odometry itself.
 *
 * Udev rule (Pi, install once) so the board gets a stable device name:
 *   echo 'KERNEL=="ttyACM*",ATTRS{idVendor}=="2341",MODE:="0666",SYMLINK+="arduino"' \
 *        | sudo tee /etc/udev/rules.d/99-arduino.rules
 *   sudo udevadm control --reload-rules && sudo udevadm trigger
 *
 * ─── Wiring (see docs/wiring_diagram_v3.svg) ──────────────────────────────
 *  4WD skid-steer: two L298N drivers, one per side, 100 RPM gear motors.
 *  L298N LEFT  : ENA=9  (PWM, front-left)  IN1=22 IN2=23
 *                ENB=10 (PWM, rear-left)   IN3=24 IN4=25
 *  L298N RIGHT : ENA=5  (PWM, front-right) IN1=26 IN2=27
 *                ENB=6  (PWM, rear-right)  IN3=28 IN4=29
 *  Door servos : left=11, right=12  (SG90, powered from 5V buck, not the Mega)
 *  HC-SR04     : TRIG=30, ECHO=31   (front-low, catches what the lidar misses)
 *  IR sensor   : 32 (digital obstacle module inside compartment, active LOW)
 *  MFRC522     : SS=53, RST=49, SCK=52, MOSI=51, MISO=50 (3.3V supply!)
 *  I2C bus     : SDA=20, SCL=21 — MPU6050 @0x68, INA219 @0x40, LCD 20x4 @0x27
 *  INA219      : high-side on the 3S 11.1V drive battery (0.1 Ω shunt)
 *
 * ─── Serial protocol (115200 baud, one ASCII line per message) ─────────────
 *  Pi → board:
 *    V,<lin m/s>,<ang rad/s>   drive command (watchdog: stop after 500 ms)
 *    O                         open compartment doors
 *    C                         close compartment doors
 *    L,<text>                  LCD line 2 (20 chars max)
 *  Board → Pi:
 *    S,READY,v2.0              boot complete (after gyro bias calibration)
 *    I,ax,ay,az,gx,gy,gz       IMU 50 Hz  (m/s², rad/s, bias-corrected gyro)
 *    U,<m>                     ultrasonic 15 Hz (-1.0 = no echo)
 *    B,<V>,<A>                 battery 1 Hz (bus volts, amps)
 *    R,<UIDHEX>                RFID tag scanned (2 s duplicate suppression)
 *    D,<0|1>                   compartment IR: 1 = item present (on change + 2 s refresh)
 *    A,DOORS,<MOVING|OPEN|CLOSED>  door actuation progress
 *    E,<msg>                   error/diagnostic
 *
 * ─── Tuning ────────────────────────────────────────────────────────────────
 *  MAX_RPM, WHEEL_* : match urdf/properties.xacro
 *  DOOR_*_OPEN/CLOSED : servo endpoint angles — set on the assembled chassis
 *  SERVO_DETACH     : detach after motion to stop SG90 buzz; set to 0 if the
 *                     doors sag under their own weight when unpowered
 */
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>

#define FW_VERSION "v3.0"

// ─── Motor driver pins (2× L298N, one per side) ────────────────────────────
#define FL_EN   9    // L298N-L ENA — front-left
#define FL_INA 22
#define FL_INB 23
#define RL_EN  10    // L298N-L ENB — rear-left
#define RL_INA 24
#define RL_INB 25
#define FR_EN   5    // L298N-R ENA — front-right
#define FR_INA 26
#define FR_INB 27
#define RR_EN   6    // L298N-R ENB — rear-right
#define RR_INA 28
#define RR_INB 29

// ─── Peripheral pins ───────────────────────────────────────────────────────
#define DOOR_L_PIN     11
#define DOOR_R_PIN     12
#define SONAR_TRIG     30
#define SONAR_ECHO     31
#define IR_PIN         32
#define RFID_SS        53
#define RFID_RST       49

// ─── Robot parameters (match URDF properties.xacro) ───────────────────────
#define WHEEL_RADIUS    0.065f   // ← TUNE: new 4WD wheels
#define WHEEL_SEP       0.330f   // ← TUNE: track width of the new chassis
#define MAX_RPM         100      // 100 RPM gear motors
#define MAX_SPEED_MPS   (MAX_RPM / 60.0f * 2.0f * 3.14159265f * WHEEL_RADIUS)

// ─── Doors ─────────────────────────────────────────────────────────────────
#define DOOR_L_OPEN     90     // ← TUNE on assembled chassis
#define DOOR_L_CLOSED    0
#define DOOR_R_OPEN     90
#define DOOR_R_CLOSED    0
#define DOOR_STEP_DEG    2     // per 20 ms tick → ~0.9 s for a 90° swing
#define SERVO_DETACH     1     // 1: detach 1 s after reaching target

// ─── I2C addresses ─────────────────────────────────────────────────────────
#define MPU_ADDR    0x68
#define INA_ADDR    0x40
#define LCD_ADDR    0x27

// ─── Timing ────────────────────────────────────────────────────────────────
#define CMD_TIMEOUT_MS     500
#define IMU_PERIOD_MS       20   // 50 Hz
#define SONAR_PERIOD_MS     66   // ~15 Hz
#define BATT_PERIOD_MS    1000
#define RFID_PERIOD_MS     100
#define IR_PERIOD_MS        50
#define IR_REFRESH_MS     2000
#define DOOR_PERIOD_MS      20
#define RFID_DEDUP_MS     2000
#define SERIAL_BAUD     115200
#define LINE_BUF_LEN        64

// ─── State ─────────────────────────────────────────────────────────────────
unsigned long last_cmd_ms = 0;
char          line_buf[LINE_BUF_LEN];
uint8_t       line_len = 0;

Servo door_l, door_r;
int   door_l_pos = DOOR_L_CLOSED, door_r_pos = DOOR_R_CLOSED;
int   door_l_tgt = DOOR_L_CLOSED, door_r_tgt = DOOR_R_CLOSED;
bool  doors_moving = false;
bool  doors_open_cmd = false;
unsigned long door_settle_ms = 0;

MFRC522 rfid(RFID_SS, RFID_RST);
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);   // 20x4 (lines 3-4 free for later)

bool  mpu_ok = false, ina_ok = false, lcd_ok = false, rfid_ok = false;
float gyro_bias[3] = {0, 0, 0};

byte          last_uid[10];
uint8_t       last_uid_len = 0;
unsigned long last_uid_ms = 0;

int  ir_state = -1;            // -1 forces first publish
unsigned long last_ir_sent_ms = 0;

unsigned long t_imu = 0, t_sonar = 0, t_batt = 0, t_rfid = 0,
              t_ir = 0, t_door = 0;

// ─── Motor control (skid-steer: left pair, right pair) ────────────────────
void driveMotor(uint8_t en, uint8_t in_a, uint8_t in_b, float vel_mps) {
    int pwm = (int)constrain(
        fabsf(vel_mps) / MAX_SPEED_MPS * 255.0f, 0.0f, 255.0f);
    if (vel_mps >= 0) {
        digitalWrite(in_a, HIGH); digitalWrite(in_b, LOW);
    } else {
        digitalWrite(in_a, LOW);  digitalWrite(in_b, HIGH);
    }
    analogWrite(en, pwm);
}

void driveSides(float v_l, float v_r) {
    driveMotor(FL_EN, FL_INA, FL_INB, v_l);
    driveMotor(RL_EN, RL_INA, RL_INB, v_l);
    driveMotor(FR_EN, FR_INA, FR_INB, v_r);
    driveMotor(RR_EN, RR_INA, RR_INB, v_r);
}

void stopMotors() {
    const uint8_t en[]  = {FL_EN, RL_EN, FR_EN, RR_EN};
    const uint8_t in[]  = {FL_INA, FL_INB, RL_INA, RL_INB,
                           FR_INA, FR_INB, RR_INA, RR_INB};
    for (uint8_t i = 0; i < 4; i++) analogWrite(en[i], 0);
    for (uint8_t i = 0; i < 8; i++) digitalWrite(in[i], LOW);
}

// ─── I2C helpers ───────────────────────────────────────────────────────────
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

// ─── MPU6050 (raw registers — no library) ──────────────────────────────────
bool mpuInit() {
    if (!i2cWrite8(MPU_ADDR, 0x6B, 0x00)) return false;  // wake
    delay(100);
    i2cWrite8(MPU_ADDR, 0x6B, 0x01);  // clock = gyro PLL
    i2cWrite8(MPU_ADDR, 0x1A, 0x03);  // DLPF 44/42 Hz (accel+gyro)
    i2cWrite8(MPU_ADDR, 0x1B, 0x00);  // gyro ±250 dps
    i2cWrite8(MPU_ADDR, 0x1C, 0x00);  // accel ±2 g
    return true;
}

// raw[0..2]=accel xyz, raw[3..5]=gyro xyz
bool mpuRead(int16_t raw[6]) {
    uint8_t b[14];
    if (!i2cReadN(MPU_ADDR, 0x3B, b, 14)) return false;
    raw[0] = (b[0] << 8) | b[1];
    raw[1] = (b[2] << 8) | b[3];
    raw[2] = (b[4] << 8) | b[5];
    raw[3] = (b[8]  << 8) | b[9];    // skip temp (b[6..7])
    raw[4] = (b[10] << 8) | b[11];
    raw[5] = (b[12] << 8) | b[13];
    return true;
}

// Robot must be stationary during boot — averages the gyro zero offset.
void mpuCalibrateGyro() {
    long sum[3] = {0, 0, 0};
    int16_t raw[6];
    const int N = 200;
    for (int i = 0; i < N; i++) {
        if (mpuRead(raw)) {
            sum[0] += raw[3]; sum[1] += raw[4]; sum[2] += raw[5];
        }
        delay(5);
    }
    for (int i = 0; i < 3; i++) gyro_bias[i] = sum[i] / (float)N;
}

void publishImu() {
    int16_t raw[6];
    if (!mpuRead(raw)) { mpu_ok = false; Serial.println(F("E,IMU_READ_FAIL")); return; }
    const float A = 9.80665f / 16384.0f;            // ±2 g  → m/s²
    const float G = (1.0f / 131.0f) * 0.01745329f;  // ±250 dps → rad/s
    Serial.print(F("I,"));
    Serial.print(raw[0] * A, 3); Serial.print(',');
    Serial.print(raw[1] * A, 3); Serial.print(',');
    Serial.print(raw[2] * A, 3); Serial.print(',');
    Serial.print((raw[3] - gyro_bias[0]) * G, 4); Serial.print(',');
    Serial.print((raw[4] - gyro_bias[1]) * G, 4); Serial.print(',');
    Serial.println((raw[5] - gyro_bias[2]) * G, 4);
}

// ─── INA219 (raw registers — no library) ───────────────────────────────────
#define INA_SHUNT_OHMS 0.1f

bool inaInit() {
    // Config: 32 V range, ±320 mV shunt gain, 12-bit, continuous
    Wire.beginTransmission(INA_ADDR);
    Wire.write(0x00); Wire.write(0x39); Wire.write(0x9F);
    return Wire.endTransmission() == 0;
}

float inaReadAmps() {
    uint8_t b[2];
    if (!i2cReadN(INA_ADDR, 0x01, b, 2)) return 0.0f;
    int16_t shunt = (b[0] << 8) | b[1];      // LSB = 10 µV
    return shunt * 10e-6f / INA_SHUNT_OHMS;
}

// Returns false on I2C failure; volts/amps valid only on true.
bool readBattery(float *volts, float *amps) {
    uint8_t b[2];
    if (!i2cReadN(INA_ADDR, 0x02, b, 2)) { ina_ok = false; return false; }
    *volts = (((b[0] << 8) | b[1]) >> 3) * 0.004f;
    *amps = inaReadAmps();
    return true;
}

// ─── LCD ───────────────────────────────────────────────────────────────────
#define LCD_COLS 20

void lcdBattLine(float volts, float amps) {
    if (!lcd_ok) return;
    char buf[LCD_COLS + 1], v[8], a[8];
    dtostrf(volts, 4, 1, v);
    dtostrf(amps, 4, 2, a);
    snprintf(buf, sizeof(buf), "Batt %sV  %sA", v, a);
    lcd.setCursor(0, 0);
    lcd.print(buf);
    for (uint8_t i = strlen(buf); i < LCD_COLS; i++) lcd.print(' ');
}

void lcdLine2(const char *text) {
    if (!lcd_ok) return;
    lcd.setCursor(0, 1);
    uint8_t n = 0;
    while (text[n] && n < LCD_COLS) { lcd.print(text[n]); n++; }
    while (n < LCD_COLS) { lcd.print(' '); n++; }
}

// ─── Ultrasonic ────────────────────────────────────────────────────────────
void publishSonar() {
    digitalWrite(SONAR_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);
    // 12 ms timeout ≈ 2 m max range; keeps the loop responsive
    unsigned long dur = pulseIn(SONAR_ECHO, HIGH, 12000UL);
    Serial.print(F("U,"));
    if (dur == 0) Serial.println(F("-1.0"));
    else          Serial.println(dur * 0.0001715f, 3);  // t/2 × 343 m/s
}

// ─── RFID ──────────────────────────────────────────────────────────────────
void pollRfid() {
    if (!rfid_ok) return;
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

    bool same = (rfid.uid.size == last_uid_len) &&
                (memcmp(rfid.uid.uidByte, last_uid, last_uid_len) == 0);
    if (same && millis() - last_uid_ms < RFID_DEDUP_MS) {
        rfid.PICC_HaltA();
        return;
    }
    memcpy(last_uid, rfid.uid.uidByte, rfid.uid.size);
    last_uid_len = rfid.uid.size;
    last_uid_ms = millis();

    Serial.print(F("R,"));
    for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] < 0x10) Serial.print('0');
        Serial.print(rfid.uid.uidByte[i], HEX);
    }
    Serial.println();
    rfid.PICC_HaltA();
}

// ─── IR compartment sensor ─────────────────────────────────────────────────
void pollIr() {
    int occupied = (digitalRead(IR_PIN) == LOW) ? 1 : 0;  // module active LOW
    if (occupied != ir_state || millis() - last_ir_sent_ms > IR_REFRESH_MS) {
        ir_state = occupied;
        last_ir_sent_ms = millis();
        Serial.print(F("D,"));
        Serial.println(occupied);
    }
}

// ─── Doors ─────────────────────────────────────────────────────────────────
void doorsCommand(bool open) {
    doors_open_cmd = open;
    door_l_tgt = open ? DOOR_L_OPEN : DOOR_L_CLOSED;
    door_r_tgt = open ? DOOR_R_OPEN : DOOR_R_CLOSED;
    if (!door_l.attached()) door_l.attach(DOOR_L_PIN);
    if (!door_r.attached()) door_r.attach(DOOR_R_PIN);
    doors_moving = true;
    Serial.println(F("A,DOORS,MOVING"));
}

int stepToward(int pos, int tgt) {
    if (pos < tgt) return min(pos + DOOR_STEP_DEG, tgt);
    if (pos > tgt) return max(pos - DOOR_STEP_DEG, tgt);
    return pos;
}

void updateDoors() {
    if (doors_moving) {
        door_l_pos = stepToward(door_l_pos, door_l_tgt);
        door_r_pos = stepToward(door_r_pos, door_r_tgt);
        door_l.write(door_l_pos);
        door_r.write(door_r_pos);
        if (door_l_pos == door_l_tgt && door_r_pos == door_r_tgt) {
            doors_moving = false;
            door_settle_ms = millis();
            Serial.println(doors_open_cmd ? F("A,DOORS,OPEN")
                                          : F("A,DOORS,CLOSED"));
        }
    } else if (SERVO_DETACH && door_l.attached() &&
               millis() - door_settle_ms > 1000) {
        door_l.detach();
        door_r.detach();
    }
}

// ─── Serial command parsing ─────────────────────────────────────────────────
void handleLine(char *line) {
    switch (line[0]) {
    case 'V': {
        if (line[1] != ',') return;
        char *p = line + 2;
        char *comma = strchr(p, ',');
        if (!comma) return;
        *comma = '\0';
        float v = atof(p);
        float w = atof(comma + 1);
        driveSides(v - w * (WHEEL_SEP / 2.0f),
                   v + w * (WHEEL_SEP / 2.0f));
        last_cmd_ms = millis();
        break;
    }
    case 'O': doorsCommand(true);  break;
    case 'C': doorsCommand(false); break;
    case 'L':
        if (line[1] == ',') lcdLine2(line + 2);
        break;
    }
}

void pollSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            line_buf[line_len] = '\0';
            handleLine(line_buf);
            line_len = 0;
        } else if (c != '\r') {
            if (line_len < LINE_BUF_LEN - 1) {
                line_buf[line_len++] = c;
            } else {
                line_len = 0;   // overflow guard: drop malformed line
            }
        }
    }
}

// ─── Setup ─────────────────────────────────────────────────────────────────
void setup() {
    const uint8_t motor_pins[] = {FL_EN, FL_INA, FL_INB, RL_EN, RL_INA,
                                  RL_INB, FR_EN, FR_INA, FR_INB, RR_EN,
                                  RR_INA, RR_INB};
    for (uint8_t i = 0; i < sizeof(motor_pins); i++)
        pinMode(motor_pins[i], OUTPUT);
    stopMotors();

    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
    pinMode(IR_PIN, INPUT_PULLUP);

    Serial.begin(SERIAL_BAUD);
    Wire.begin();
    Wire.setClock(400000);

    SPI.begin();
    rfid.PCD_Init();
    // Version reg reads 0x00/0xFF when the reader is absent/miswired
    byte ver = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    rfid_ok = (ver != 0x00 && ver != 0xFF);
    if (!rfid_ok) Serial.println(F("E,RFID_NOT_FOUND"));

    Wire.beginTransmission(LCD_ADDR);
    lcd_ok = (Wire.endTransmission() == 0);
    if (lcd_ok) {
        lcd.init();
        lcd.backlight();
        lcdLine2("OfficeMate " FW_VERSION);
    } else {
        Serial.println(F("E,LCD_NOT_FOUND"));
    }

    ina_ok = inaInit();
    if (!ina_ok) Serial.println(F("E,INA219_NOT_FOUND"));

    mpu_ok = mpuInit();
    if (mpu_ok) {
        mpuCalibrateGyro();     // ~1 s, robot must be stationary
    } else {
        Serial.println(F("E,MPU6050_NOT_FOUND"));
    }

    // Doors start closed
    door_l.attach(DOOR_L_PIN); door_r.attach(DOOR_R_PIN);
    door_l.write(DOOR_L_CLOSED); door_r.write(DOOR_R_CLOSED);
    door_settle_ms = millis();

    last_cmd_ms = millis();
    Serial.println(F("S,READY," FW_VERSION));
}

// ─── Loop ──────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    pollSerial();

    if (now - last_cmd_ms > CMD_TIMEOUT_MS) stopMotors();

    if (mpu_ok && now - t_imu >= IMU_PERIOD_MS) {
        t_imu = now;
        publishImu();
    }
    if (now - t_sonar >= SONAR_PERIOD_MS) {
        t_sonar = now;
        publishSonar();
    }
    if (now - t_ir >= IR_PERIOD_MS) {
        t_ir = now;
        pollIr();
    }
    if (now - t_rfid >= RFID_PERIOD_MS) {
        t_rfid = now;
        pollRfid();
    }
    if (ina_ok && now - t_batt >= BATT_PERIOD_MS) {
        t_batt = now;
        float volts, amps;
        if (readBattery(&volts, &amps)) {
            Serial.print(F("B,"));
            Serial.print(volts, 2); Serial.print(',');
            Serial.println(amps, 2);
            lcdBattLine(volts, amps);
        }
    }
    if (now - t_door >= DOOR_PERIOD_MS) {
        t_door = now;
        updateDoors();
    }
}
