#include "kinematics.h"

BaseDrive::BaseDrive(DriveType type, float chassis_L, float chassis_W) {
    driveType = type;
    chassis_length = chassis_L;
    chassis_breadth = chassis_W;
}

void BaseDrive::attachMotors(Actuator::Motor* m1,
                             Actuator::Motor* m2,
                             Actuator::Motor* m3,
                             Actuator::Motor* m4)
{
    motor1 = m1;
    motor2 = m2;
    motor3 = m3;
    motor4 = m4;
}

void BaseDrive::setVelocity(float vx_, float vy_, float vw_) {
    vx = vx_;
    vy = vy_;
    vw = vw_;
}

void BaseDrive::computeIK() {

    switch(driveType) {

        case TRUCK_TIRE:
            differentialIK();
            break;

        case MECANUM_WHEEL:
            mecanumIK();
            break;

        default:
            break;
    }
}

void BaseDrive::applyPWM()
{
    if(motor1) motor1->set_Speed(wheel1);
    if(motor2) motor2->set_Speed(wheel2);
    if(motor3) motor3->set_Speed(wheel3);
    if(motor4) motor4->set_Speed(wheel4);
}

void BaseDrive::applyPWM(int pwm1,int pwm2,int pwm3,int pwm4){
    motor1->set_PWM(pwm1);
    motor2->set_PWM(pwm2);
    // motor3->set_PWM(pwm3);
    // motor4->set_PWM(pwm4);
}

void BaseDrive::update(float vx, float vy, float vw){
    setVelocity(vx,vy,vw);
    computeIK();
    applyPWM();
}

void BaseDrive::differentialIK()
{
    float L = chassis_length;

    wheel1 = vx - vw * L/2;
    wheel2 = vx + vw * L/2;
}

void BaseDrive::mecanumIK()
{
    float L = chassis_length;
    float W = chassis_breadth;

    wheel1 = vx - vy - vw*(L+W);
    wheel2 = vx + vy + vw*(L+W);
    wheel3 = vx + vy - vw*(L+W);
    wheel4 = vx - vy + vw*(L+W);
}