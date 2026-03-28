// // #include <Arduino.h>
// // #include "motor.h"
// // #include "kinematics.h"
// // #include "sensor.h"

// // struct VelocityCmd {
// //     float vx = 0.0;
// //     float vy = 0.0;
// //     float vw = 0.0;
// // };

// // Actuator::Motor M1(25, 26);
// // Actuator::Motor M2(27, 14);
// // BaseDrive drive(BaseDrive::TRUCK_TIRE);
// // SensorManager sensor{};
// // SensorData D;

// // VelocityCmd vel;

// // void control_loop(void *pvParameters);
// // void setup() {
// //   Serial.begin(115200);

// //   drive.attachMotors(&M1, &M2);
// //   sensor.begin();

// // //   xTaskCreatePinnedToCore(control_loop, "BASE CONTROL", 8192, NULL, 1, NULL, 0);
// //   xTaskCreatePinnedToCore(control_loop, "pid_control", 4096, NULL, 1, NULL, 1);
// // }

// // void loop() {
// //     // drive.update(200, 0, 0);
  
// // //   Serial.println("11111111111");
// // // // send sensor data
// // sensor.update();
// //   D = sensor.getData();
// // //   drive.update(200, 0, 0);
// // //   Serial.write(0xAA);
// // //   Serial.write((uint8_t*)&D, sizeof(D));

// // //     //Receive velocity command 
// // //     if (Serial.available() >= (1 + sizeof(VelocityCmd))) {

// // //     // if (Serial.peek() == 0xAA) {

// // //         Serial.read(); // remove header
// // //         Serial.readBytes((uint8_t*)&vel, sizeof(VelocityCmd));
// // //       }
// //     // }
// //     delay(10);
// // }

// // void control_loop(void *pvParameters) {
// //   for (;;) {
// //     drive.update(0.8, 0, 0);
// //     // M1.set_PWM(200);
// //     // M2.set_PWM(200);
// //     //  Serial.println("00000000000");
// //     vTaskDelay(10);
// //   }
// // }

#include <Arduino.h>
#include "motor.h"
#include "kinematics.h"
#include "sensor.h"

// ─── Ultrasonic pins ──────────────────────────────────────────────────────────
#define US1_TRIG  13
#define US1_ECHO  18
#define US2_TRIG  19
#define US2_ECHO  23

// ─── Objects ──────────────────────────────────────────────────────────────────
struct VelocityCmd {
    float vx = 0.0;
    float vy = 0.0;
    float vw = 0.0;
};

Actuator::Motor M1(25, 26);
Actuator::Motor M2(27, 14);
BaseDrive        drive(BaseDrive::TRUCK_TIRE);
SensorManager    sensor{};
SensorData       D;
VelocityCmd      vel;

// ─── RTOS handles 
SemaphoreHandle_t us_mutex;     // protects D.ultrasonic_distance / D.water_distance
SemaphoreHandle_t vel_mutex;    // protects vel struct (written by loop, read by control_loop)

// ─── Echo ISR state
volatile uint32_t us1_start = 0, us1_pulse = 0;
volatile uint32_t us2_start = 0, us2_pulse = 0;
volatile bool     us1_ready = false, us2_ready = false;

// ─── Serial RX state machine
static uint8_t rx_buf[sizeof(VelocityCmd)];
static int     rx_count = 0;
static bool    rx_header_found = false;

// ─── ISRs 
void IRAM_ATTR us1_echo_isr() {
    uint32_t now = micros();
    if (gpio_get_level((gpio_num_t)US1_ECHO)) {
        us1_start = now;
    } else {
        us1_pulse = now - us1_start;
        if (us1_pulse > 38000) us1_pulse = 0;   // HC-SR04 max ~38ms
        us1_ready = true;
    }
}

void IRAM_ATTR us2_echo_isr() {
    uint32_t now = micros();
    if (gpio_get_level((gpio_num_t)US2_ECHO)) {
        us2_start = now;
    } else {
        us2_pulse = now - us2_start;
        if (us2_pulse > 60000) us2_pulse = 0;   // JSN-SR04T max range clamp
        us2_ready = true;
    }
}

inline float pulse_to_cm(uint32_t us) {
    return (us * 0.0343f) / 2.0f;
}

void ultrasonic_task(void *pvParameters) {
    for (;;) {

        // ── US1 (HC-SR04) ──
        detachInterrupt(digitalPinToInterrupt(US2_ECHO));
        us1_ready = false;

        digitalWrite(US1_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US1_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US1_TRIG, LOW);

        vTaskDelay(pdMS_TO_TICKS(40));

        if (us1_ready) {
            float dist = pulse_to_cm(us1_pulse);
            xSemaphoreTake(us_mutex, portMAX_DELAY);
                D.ultrasonic_distance = dist;
            xSemaphoreGive(us_mutex);
        }

        attachInterrupt(digitalPinToInterrupt(US2_ECHO), us2_echo_isr, CHANGE);
        vTaskDelay(pdMS_TO_TICKS(10));

        // ── US2 (JSN-SR04T) ──
        detachInterrupt(digitalPinToInterrupt(US1_ECHO));
        us2_ready = false;

        digitalWrite(US2_TRIG, LOW);  delayMicroseconds(2);
        digitalWrite(US2_TRIG, HIGH); delayMicroseconds(10);
        digitalWrite(US2_TRIG, LOW);

        vTaskDelay(pdMS_TO_TICKS(75));

        if (us2_ready) {
            float dist = pulse_to_cm(us2_pulse);
            xSemaphoreTake(us_mutex, portMAX_DELAY);
                D.water_distance = dist;
            xSemaphoreGive(us_mutex);
        }

        attachInterrupt(digitalPinToInterrupt(US1_ECHO), us1_echo_isr, CHANGE);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void sensor_send(void *pvParameters) {
    for (;;) {
            // ── Sensor read ──
    sensor.update();
    SensorData latest = sensor.getData();

    xSemaphoreTake(us_mutex, portMAX_DELAY);
        latest.ultrasonic_distance = D.ultrasonic_distance;
        latest.water_distance      = D.water_distance;
    xSemaphoreGive(us_mutex);

    Serial.write(0xAA);
    Serial.write((uint8_t*)&latest, sizeof(SensorData));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── Task: Control loop (Core 1, priority 3) ──────────────────────────────────
void control_loop(void *pvParameters) {
    VelocityCmd cmd{};
    for (;;) {
        xSemaphoreTake(vel_mutex, portMAX_DELAY);
            cmd = vel;
        xSemaphoreGive(vel_mutex);

        drive.update(cmd.vx, cmd.vy, cmd.vw);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Ultrasonic GPIO
    pinMode(US1_TRIG, OUTPUT); pinMode(US1_ECHO, INPUT);
    pinMode(US2_TRIG, OUTPUT); pinMode(US2_ECHO, INPUT);
    attachInterrupt(digitalPinToInterrupt(US1_ECHO), us1_echo_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(US2_ECHO), us2_echo_isr, CHANGE);

    // Motors & sensors
    drive.attachMotors(&M1, &M2);
    sensor.begin();

    // Mutexes
    us_mutex  = xSemaphoreCreateMutex();
    vel_mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(control_loop,    "CONTROL",    4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ultrasonic_task, "ULTRASONIC", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensor_send, "sensor", 2048, NULL, 2, NULL, 0);

}

void loop() {

    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (!rx_header_found) {
            if (b == 0xAA) rx_header_found = true;   // wait for header byte
        } else {
            rx_buf[rx_count++] = b;

            if (rx_count == sizeof(VelocityCmd)) {
                // Full packet received — write to vel under mutex
                xSemaphoreTake(vel_mutex, portMAX_DELAY);
                    memcpy(&vel, rx_buf, sizeof(VelocityCmd));
                xSemaphoreGive(vel_mutex);

                // Reset parser for next packet
                rx_count        = 0;
                rx_header_found = false;
            }
        }
    }

    delay(5);
}
