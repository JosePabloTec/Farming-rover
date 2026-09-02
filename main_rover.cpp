#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>

#define MPU6050 0x68

// =====================================================
// Gyroscope
// =====================================================

float ax, ay, az;

float vx = 0;
float vy = 0;
float vz = 0;

float px = 0;
float py = 0;
float pz = 0;

float gx, gy, gz;

float roll = 0;
float pitch = 0;
float yaw = 0;

unsigned long gyroPreviousTime;
float gyroDt;


// =====================================================
// Ultrasonic sensor
// =====================================================

const int TRIG = A0;
const int ECHO = A1;


// =====================================================
// PI Controller
// =====================================================

const float Kp = 4.0;
const float Ki = 0.5;


// =====================================================
// Motor limits
// =====================================================

const int MAX_SPEED = 255;

const float INTEGRAL_LIMIT = 100.0;


// =====================================================
// Goal parameters
// =====================================================

const float interval = 50.0;
const float threshold = 5.0;

const unsigned long STOP_TIME = 3000;
const unsigned long ROTATE_TIME = 750;


// =====================================================
// Motors
// =====================================================

// M1 = front left
// M2 = rear left
// M3 = front right
// M4 = rear right

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);


// =====================================================
// Variables
// =====================================================

float distance = 0.0;
float target = 0.0;
float error = 0.0;

float integral = 0.0;

bool target_reached = false;

int goal_number = 1;

unsigned long previousTime;


// =====================================================
// Function declarations
// =====================================================

float getDistance();
float set_target(float current_distance);

void rotate();
void forward();
void stopmotors();

float read_gyroscope();
void reset_yaw();


// =====================================================
// Setup
// =====================================================

void setup() {

  // Initial motor speed
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  digitalWrite(TRIG, LOW);


  // ---------------------------------------------------
  // Initialize MPU6050
  // ---------------------------------------------------

  Wire.begin();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();


  // ---------------------------------------------------
  // Initial 10 second delay
  // ---------------------------------------------------

  delay(10000);


  // ---------------------------------------------------
  // Initialize gyro timer
  // ---------------------------------------------------

  gyroPreviousTime = micros();


  // ---------------------------------------------------
  // Measure initial distance
  // ---------------------------------------------------

  distance = getDistance();

  if (distance > 0) {

    target = distance - interval;
  }

  previousTime = millis();
}


// =====================================================
// Motor functions
// =====================================================

void rotate() {

  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}


void forward() {

  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}


void stopmotors() {

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


// =====================================================
// Ultrasonic sensor
// =====================================================

float getDistance() {

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);

  if (duration == 0) {

    return -1;
  }

  float distance = duration * 0.0343 / 2.0;

  return distance;
}


// =====================================================
// Set next target
// =====================================================

float set_target(float current_distance) {

  return current_distance - interval;
}


// =====================================================
// Gyroscope
// =====================================================

float read_gyroscope() {

  unsigned long currentTime = micros();

  gyroDt = (currentTime - gyroPreviousTime) / 1000000.0;

  gyroPreviousTime = currentTime;


  int16_t rawAx, rawAy, rawAz;
  int16_t rawGx, rawGy, rawGz;


  Wire.beginTransmission(MPU6050);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050, 14);


  rawAx = Wire.read() << 8 | Wire.read();
  rawAy = Wire.read() << 8 | Wire.read();
  rawAz = Wire.read() << 8 | Wire.read();

  // Temperature
  Wire.read();
  Wire.read();

  rawGx = Wire.read() << 8 | Wire.read();
  rawGy = Wire.read() << 8 | Wire.read();
  rawGz = Wire.read() << 8 | Wire.read();


  // ---------------------------------------------------
  // Convert accelerometer
  // ---------------------------------------------------

  ax = (rawAx / 16384.0) * 9.81;
  ay = (rawAy / 16384.0) * 9.81;
  az = (rawAz / 16384.0) * 9.81;

  az = az - 9.81;


  // ---------------------------------------------------
  // Convert gyroscope
  // ---------------------------------------------------

  gx = rawGx / 131.0;
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;


  // ---------------------------------------------------
  // Integrate yaw
  // ---------------------------------------------------

  roll  = roll + gx * gyroDt;
  pitch = pitch + gy * gyroDt;

  yaw = yaw + gz * gyroDt;


  return yaw;
}


// =====================================================
// Reset yaw
// =====================================================

void reset_yaw() {

  yaw = 0;

  gyroPreviousTime = micros();
}


// =====================================================
// Main loop
// =====================================================

void loop() {


  // ===================================================
  // Measure distance
  // ===================================================

  distance = getDistance();


  // ===================================================
  // Read gyroscope
  // ===================================================

  yaw = read_gyroscope();


  // ===================================================
  // Invalid ultrasonic reading
  // ===================================================

  if (distance <= 0) {

    stopmotors();

    delay(100);

    return;
  }


  // ===================================================
  // ROTATION CONDITION
  // ===================================================

  if (distance <= interval) {

    stopmotors();

    delay(STOP_TIME);


    // -------------------------------------------------
    // Rotate
    // -------------------------------------------------

    motor1.setSpeed(255);
    motor2.setSpeed(255);
    motor3.setSpeed(255);
    motor4.setSpeed(255);

    rotate();

    delay(ROTATE_TIME);

    stopmotors();


    // -------------------------------------------------
    // Reset yaw AFTER rotation
    // -------------------------------------------------

    reset_yaw();


    // -------------------------------------------------
    // Measure distance after rotation
    // -------------------------------------------------

    delay(200);

    distance = getDistance();


    if (distance > 0) {

      // -----------------------------------------------
      // Set new target
      // -----------------------------------------------

      target = set_target(distance);

      goal_number++;


      // -----------------------------------------------
      // Reset integral
      // -----------------------------------------------

      integral = 0.0;

      previousTime = millis();

      target_reached = false;
    }

    return;
  }


  // ===================================================
  // TARGET REACHED
  // ===================================================

  if (target_reached) {

    integral = 0.0;

    distance = getDistance();


    if (distance > 0) {

      target = set_target(distance);

      goal_number++;

      target_reached = false;

      previousTime = millis();
    }

    return;
  }


  // ===================================================
  // MOVING
  // ===================================================

  // ---------------------------------------------------
  // Calculate dt
  // ---------------------------------------------------

  unsigned long currentTime = millis();

  float dt = (currentTime - previousTime) / 1000.0;

  previousTime = currentTime;


  // ---------------------------------------------------
  // Error to NEXT GOAL
  // ---------------------------------------------------

  error = distance - target;


  // ---------------------------------------------------
  // Integral
  // ---------------------------------------------------

  integral += error * dt;


  // ---------------------------------------------------
  // Anti-windup
  // ---------------------------------------------------

  integral = constrain(
    integral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT
  );


  // ---------------------------------------------------
  // PI Controller
  // ---------------------------------------------------

  float control = Kp * error + Ki * integral;


  // ---------------------------------------------------
  // Motor velocity
  // ---------------------------------------------------

  int velocity;


  if (error > 15.0) {

    velocity = 200;

  }

  else {

    velocity = constrain(
      control,
      75,
      MAX_SPEED
    );
  }


  // ===================================================
  // GYROSCOPE YAW CORRECTION
  // ===================================================

  int leftSpeed = velocity;
  int rightSpeed = velocity;


  if (yaw > 10.0) {

    // Right wheels +30
    rightSpeed = velocity + 30;
  }

  else if (yaw < -10.0) {

    // Left wheels +30
    leftSpeed = velocity + 30;
  }


  // ---------------------------------------------------
  // Limit speeds
  // ---------------------------------------------------

  leftSpeed = constrain(
    leftSpeed,
    0,
    MAX_SPEED
  );

  rightSpeed = constrain(
    rightSpeed,
    0,
    MAX_SPEED
  );


  // ---------------------------------------------------
  // Apply motor speeds
  // ---------------------------------------------------

  motor1.setSpeed(leftSpeed);
  motor2.setSpeed(leftSpeed);

  motor3.setSpeed(rightSpeed);
  motor4.setSpeed(rightSpeed);


  // ---------------------------------------------------
  // Move forward
  // ---------------------------------------------------

  forward();


  // ===================================================
  // CHECK IF TARGET WAS REACHED
  // ===================================================

  if (abs(error) < threshold) {

    stopmotors();

    // -------------------------------------------------
    // Stop
    // -------------------------------------------------

    delay(STOP_TIME);


    // -------------------------------------------------
    // Create next target immediately
    // -------------------------------------------------

    distance = getDistance();


    if (distance > 0) {

      target = set_target(distance);

      goal_number++;

      integral = 0.0;

      target_reached = false;

      previousTime = millis();
    }
  }


  delay(20);
}