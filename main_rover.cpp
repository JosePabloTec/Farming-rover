#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>
#include <LIS3MDL.h>

LIS3MDL mag;
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

int right_yaw = 0;
int base_vel = 180;
const float Kpyaw = 0.07;
float yaw;
float error;
int correction;

unsigned long previousMotorMillis = 0;
unsigned long previousPrintMillis = 0;
const unsigned long motorInterval = 300;
const unsigned long printInterval = 100;

bool motors_on = false;

void set_speed(int velocity, int delta) {
    motor1.setSpeed(constrain(velocity - delta, 0, 255));
    motor2.setSpeed(constrain(velocity - delta, 0, 255));
    motor3.setSpeed(constrain(velocity + delta, 0, 255));
    motor4.setSpeed(constrain(velocity + delta, 0, 255));
}

void stopmotors() {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
}

void forward(int velocity, int delta) {
    set_speed(velocity, delta);
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(BACKWARD);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!mag.init()) {
        Serial.println("LIS3MDL not detected!");
        while (1);
    }

    mag.enableDefault();
    Serial.println("Magnetometer initialized");
    delay(10000);
    mag.read();
    right_yaw = mag.m.z;
    previousMotorMillis = millis();
    previousPrintMillis = millis();
}

void loop() {
    unsigned long currentMillis = millis();

    mag.read();
    yaw = mag.m.z;
    error = yaw - right_yaw;
    correction = round(error * Kpyaw*0.5);

    if (abs(correction) < 10) {
        correction = 0;
    }

    if (correction > 0){
      correction = correction*1.5;
    }

    if (motors_on) {
        forward(base_vel, correction);
    }

    if (currentMillis - previousMotorMillis >= motorInterval) {
        previousMotorMillis = currentMillis;
        motors_on = !motors_on;

        if (motors_on) {
            forward(base_vel, correction);
        }
        else {
            stopmotors();
        }
    }

    if (currentMillis - previousPrintMillis >= printInterval) {
        previousPrintMillis = currentMillis;
        Serial.println(correction);

    }
}