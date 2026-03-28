#pragma pack(push, 1)
struct SensorData {

    float sharp_ir_distance;
    float water_distance;
    float ultrasonic_distance;
    
    uint8_t hall1;
    uint8_t hall2;
    // // uint8_t ir_left;
    // // uint8_t ir_right;
    uint8_t limit_switch_1;
    uint8_t limit_switch_2;

    int32_t encoder_left;
    int32_t encoder_right;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;

    float pitch;
    float roll;

    uint8_t mpu_available;

    float ch0, ch1, ch2, ch3 ;
};
#pragma pack(pop)
