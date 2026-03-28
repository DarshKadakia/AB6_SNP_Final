#include "sensor.h"
#include <Arduino.h>
#include "Wire.h"

void SensorManager::begin() {

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(10);

    // Set P0–P6 as inputs
    pcfState = 0xFFF;  
    writePCF(pcfState);

    ads.begin(0x48);
    // Configure ADS1115
    ads.setGain(GAIN_ONE);                    // ±4.096V range → 0.125mV/bit
    ads.setDataRate(RATE_ADS1115_128SPS);

    encoderLeft.attachHalfQuad(ENCODER_LEFT_A, ENCODER_LEFT_B),
    encoderRight.attachHalfQuad(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

    pinMode(SHARP_IR_PIN, INPUT);
    
    if (mpu.begin()) {

        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

        data.mpu_available = true;

    } else {
        data.mpu_available = false;
    }
}

void SensorManager::update() {

    readDistanceSensors();
    readDigitalSensors();
    readEncoders();
    readIMU();
    computeOrientation();
    readADC();
}

const SensorData& SensorManager::getData() const {
    return data;
}

void SensorManager::readDigitalSensors() {
    uint16_t inputState = readPCF();
    data.limit_switch_1 = !(inputState & (1 << LIMIT1_PIN));
    data.limit_switch_2 = !(inputState & (1 << LIMIT2_PIN));

//     data.ir_left  = !(inputState & (1 << IR_LEFT_PIN));
//     data.ir_right = !(inputState & (1 << IR_RIGHT_PIN));

    data.hall1 = !(inputState & (1 << HALL1_PIN));
    data.hall2 = !(inputState & (1 << HALL2_PIN));
}

void SensorManager::readEncoders() {
    data.encoder_left = encoderLeft.getCount();
    data.encoder_right = encoderRight.getCount();
}

void SensorManager::resetEncoders() {

    encoderLeft.clearCount();
    encoderRight.clearCount();
}

void SensorManager::readDistanceSensors() {
//     data.water_distance = readUltrasonic(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
//     if (data.water_distance == 0) data.water_distance = -1; // Out of range
    
//     // Ultrasonic rear
//     data.ultrasonic_distance = readUltrasonic(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
//     if (data.ultrasonic_distance == 0) data.ultrasonic_distance = -1;

    int raw = analogRead(SHARP_IR_PIN);
    // float volts = raw * 0.0048828125;  // value from sensor * (5/1024)
    // float distance = 13*pow(volts, -1); // worked out from datasheet graph
    // delay(1000); // slow down serial port 
    
    // if (distance <= 30){
        // Serial.println(distance);   // print the distance
    // }
    float voltage = raw * (3.3 / 4095.0); 
    float distance = 27.86 * pow(voltage, -1.15);
    
//     // Clamp to valid range
//     if (distance < 10 || distance > 80) {
//         distance = -1;
//         return; // Out of range
//     }
    
    data.sharp_ir_distance = distance;
}

// // long SensorManager::readUltrasonic(uint8_t trig, uint8_t echo)
// // {
// //     digitalWrite(trig, LOW);
// //     delayMicroseconds(2);

// //     digitalWrite(trig, HIGH);
// //     delayMicroseconds(10);

// //     digitalWrite(trig, LOW);

// //     long duration = pulseIn(echo, HIGH, 30000);  // 30ms timeout

// //     if (duration == 0) return -1;

// //     return duration * 0.034 / 2;  // cm
// // }

void SensorManager::readIMU() {

    if (!data.mpu_available) return;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    data.accel_x = a.acceleration.x;
    data.accel_y = a.acceleration.y;
    data.accel_z = a.acceleration.z;

    data.gyro_x = g.gyro.x;
    data.gyro_y = g.gyro.y;
    data.gyro_z = g.gyro.z;

    data.temperature = temp.temperature;
}

void SensorManager::computeOrientation() {

    // simple complementary example
    data.pitch = atan2(data.accel_y, data.accel_z);
    data.roll  = atan2(data.accel_x, data.accel_z);
}
void SensorManager::writePCF(uint16_t value){
    Wire.beginTransmission(this ->PCF_ADDR);
    Wire.write(lowByte(value));
    Wire.write(highByte(value));
    Wire.endTransmission();
}
uint16_t SensorManager::readPCF(){
    Wire.requestFrom(this->PCF_ADDR, 2);
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    return (high << 8) | low;
}

void SensorManager::readADC() {
  // Each readADC_SingleEnded() triggers one conversion (~8ms at 128SPS)
  float multiplier = ads.computeVolts(1);   // volts per bit at current gain

  data.ch0 = ads.computeVolts(ads.readADC_SingleEnded(0));
  data.ch1 = ads.computeVolts(ads.readADC_SingleEnded(1));
  data.ch2 = ads.computeVolts(ads.readADC_SingleEnded(2));
  data.ch3 = ads.computeVolts(ads.readADC_SingleEnded(3));
}