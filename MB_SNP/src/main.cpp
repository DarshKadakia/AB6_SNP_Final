#include <Arduino.h>
#include "motor.h"
#include "kinematics.h"
#include "sensor.h"

// ─── Ultrasonic pins ──────────────────────────────────────────────────────────
#define US1_TRIG  18
#define US1_ECHO  13
#define US2_TRIG  19
#define US2_ECHO  23

// ─── Objects ──────────────────────────────────────────────────────────────────
Actuator::Motor M1(25, 26);
Actuator::Motor M2(27, 14);
BaseDrive        drive(BaseDrive::TRUCK_TIRE);
SensorManager    sensor{};

// ─── Shared velocity (Core 0 writes, Core 1 doesn't touch) ───────────────────
// No mutex needed — only one writer, floats are atomic on ESP32
volatile float vx = 0, vy = 0, vw = 0;

// ─── Serial RX parser ─────────────────────────────────────────────────────────
struct VelocityCmd { float vx, vy, vw; };
static uint8_t rx_buf[sizeof(VelocityCmd)];
static int     rx_count = 0;
static bool    rx_header_found = false;

// ─── Ultrasonic ISRs ──────────────────────────────────────────────────────────
volatile uint32_t us1_start = 0, us1_pulse = 0;
volatile uint32_t us2_start = 0, us2_pulse = 0;
volatile bool     us1_ready = false, us2_ready = false;

void IRAM_ATTR us1_echo_isr() {
    uint32_t now = micros();
    if (gpio_get_level((gpio_num_t)US1_ECHO)) {
        us1_start = now;
    } else {
        us1_pulse = now - us1_start;
        if (us1_pulse > 38000) us1_pulse = 0;
        us1_ready = true;
    }
}

void IRAM_ATTR us2_echo_isr() {
    uint32_t now = micros();
    if (gpio_get_level((gpio_num_t)US2_ECHO)) {
        us2_start = now;
    } else {
        us2_pulse = now - us2_start;
        if (us2_pulse > 60000) us2_pulse = 0;
        us2_ready = true;
    }
}

inline float pulse_to_cm(uint32_t us) {
    return (us * 0.0343f) / 2.0f;
}

// ─── Core 1 task: sensor read + serial TX ────────────────────────────────────
void sensor_tx_task(void *) {
    for (;;) {
        // Fire US1, wait for echo ISR
        us1_ready = false;
        digitalWrite(US1_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US1_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US1_TRIG, LOW);
        vTaskDelay(pdMS_TO_TICKS(40));    // HC-SR04 max roundtrip

        // Fire US2, wait for echo ISR
        us2_ready = false;
        digitalWrite(US2_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US2_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US2_TRIG, LOW);
        vTaskDelay(pdMS_TO_TICKS(75));    // JSN-SR04T max roundtrip

        // Read IMU / other sensors
        sensor.update();
        SensorData D = sensor.getData();

        // Merge ultrasonic results
        if (us1_ready) D.ultrasonic_distance = pulse_to_cm(us1_pulse);
        if (us2_ready) D.water_distance      = pulse_to_cm(us2_pulse);

        // Send over serial
        Serial.write(0xAA);
        Serial.write((uint8_t*)&D, sizeof(SensorData));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(US1_TRIG, OUTPUT); pinMode(US1_ECHO, INPUT);
    pinMode(US2_TRIG, OUTPUT); pinMode(US2_ECHO, INPUT);
    attachInterrupt(digitalPinToInterrupt(US1_ECHO), us1_echo_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(US2_ECHO), us2_echo_isr, CHANGE);

    drive.attachMotors(&M1, &M2);
    sensor.begin();

    // Sensor read + TX lives on Core 1
    xTaskCreatePinnedToCore(sensor_tx_task, "SENSOR_TX", 4096, NULL, 2, NULL, 1);
}

// ─── Core 0: serial RX + motor control ───────────────────────────────────────
void loop() {
    if (Serial.available()) {
        uint8_t b = Serial.read();
        if (!rx_header_found) {
            if (b == 0xAA) rx_header_found = true;
        } else {
            rx_buf[rx_count++] = b;
            if (rx_count == sizeof(VelocityCmd)) {
                VelocityCmd cmd;
                memcpy(&cmd, rx_buf, sizeof(VelocityCmd));
                vx = cmd.vx; vy = cmd.vy; vw = cmd.vw;
                rx_count = 0;
                rx_header_found = false;
            }
        }
    }
    drive.update(vx, vy, vw);
    // delay(10);
}