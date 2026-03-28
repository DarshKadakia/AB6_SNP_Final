#pragma once
#include<Arduino.h>
#include "Wire.h"
#include "SensorData.h"
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>

class SensorManager {

public:
//PCF8575 - IO EXPANDER
    static constexpr uint8_t I2C_SDA = 21;
    static constexpr uint8_t I2C_SCL = 22;
    static constexpr uint8_t PCF_ADDR = 0x20;
    // // I2C mpu
    static constexpr uint8_t MPU6050_ADDR = 0x68;

    // // Distance Sensors
    // static constexpr uint8_t WATER_SENSOR_TRIG = 5;     // JSN-SR04T trigger (front)
    // static constexpr uint8_t WATER_SENSOR_ECHO = 18;    // JSN-SR04T echo (front)
    // static constexpr uint8_t ULTRASONIC_TRIG = 19;    // HC-SR04 trigger (rear)
    // static constexpr uint8_t ULTRASONIC_ECHO = 21;    // HC-SR04 echo (rear)
    static constexpr uint8_t SHARP_IR_PIN = 33;    // Sharp IR analog pin (rear)

    // // Digital Sensors
    static constexpr uint8_t HALL1_PIN = 0;    // Hall effect sensor 1 (bottom left)
    static constexpr uint8_t HALL2_PIN = 1;   // Hall effect sensor 2 (bottom right)
    // static constexpr uint8_t IR_LEFT_PIN = 5;    // IR proximity left side
    // static constexpr uint8_t IR_RIGHT_PIN = 6;    // IR proximity right side
    static constexpr uint8_t LIMIT1_PIN = 5;     // Limit switch front
    static constexpr uint8_t LIMIT2_PIN = 6;

    // Encoders
    static constexpr uint8_t ENCODER_LEFT_A = 34;    // Left encoder channel A
    static constexpr uint8_t ENCODER_LEFT_B = 39;   // Left encoder channel B
    static constexpr uint8_t ENCODER_RIGHT_A = 32;    // Right encoder channel A
    static constexpr uint8_t ENCODER_RIGHT_B = 35;    // Right encoder channel B

    // NeoPixel LEDs
    // static constexpr uint8_t NEOPIXEL_PIN = 2;     // WS2812B data pin
    // static constexpr uint8_t NUM_LEDS = 4;     // Number of LEDs

    // // ===== CONSTANTS =====
    // static constexpr uint16_t MAX_DISTANCE = 450;
    // static constexpr uint32_t SERIAL_BAUD = 115200;

    // SensorManager();

    void begin();
    void update();

    const SensorData& getData() const;

    void resetEncoders();

private:
    SensorData data;

    uint16_t pcfState = 0xFFFF;

    ESP32Encoder encoderLeft;
    ESP32Encoder encoderRight;


    Adafruit_MPU6050 mpu;
    Adafruit_ADS1115 ads;

    void readDistanceSensors();
    void readDigitalSensors();
    void readEncoders();
    void readIMU();
    void computeOrientation();
    long readUltrasonic(uint8_t trig, uint8_t echo);

    void writePCF(uint16_t value);
    uint16_t readPCF();

    void readADC();
};