#include <Arduino.h>
#include <AFMotor.h>

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

const int MAX_SPEED = 255;

// Maximum allowed integral contribution
const float INTEGRAL_LIMIT = 100.0;


// =====================================================
// Goal parameters
// =====================================================

const float interval = 50.0;     // Distance between goals [cm]
const float threshold = 5.0;     // Target tolerance [cm]

const unsigned long STOP_TIME = 5000;  // 5 seconds


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


// Time used for integral
unsigned long previousTime;


// =====================================================
// Setup
// =====================================================

void setup() {

  // Initial motor speed
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);

  Serial.begin(9600);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  digitalWrite(TRIG, LOW);

  // 10 second delay
  delay(10000);


  // ---------------------------------------------
  // Measure initial distance
  // ---------------------------------------------

  distance = getDistance();

  if (distance > 0) {

    target = distance - interval;

    Serial.println("================================");
    Serial.println("ROBOT STARTED");
    Serial.println("================================");

    Serial.print("Initial distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    Serial.print("Goal 1: ");
    Serial.print(target);
    Serial.println(" cm");
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

    Serial.println("NO ECHO");

    return -1;
  }

  float distance = duration * 0.0343 / 2.0;

  return distance;
}


// =====================================================
// Set next target
// =====================================================

float set_target(float current_distance) {

  target = current_distance - interval;

  return target;
}


// =====================================================
// Main loop
// =====================================================

void loop() {

  // ---------------------------------------------------
  // Measure distance
  // ---------------------------------------------------

  distance = getDistance();


  // ---------------------------------------------------
  // Invalid ultrasonic reading
  // ---------------------------------------------------

  if (distance <= interval) {

    stopmotors();
      
    delay(5000);
    motor1.setSpeed(255);
    motor2.setSpeed(255);
    motor3.setSpeed(255);
    motor4.setSpeed(255);
    rotate();
    delay(1500);
    stopmotors();
  }


  // ===================================================
  // TARGET REACHED
  // ===================================================

  if (target_reached) {

    // -----------------------------------------------
    // Reset integral for new goal
    // -----------------------------------------------

    integral = 0.0;

    // -----------------------------------------------
    // Create next target
    // -----------------------------------------------

    target = set_target(distance);

    goal_number++;

    target_reached = false;

    previousTime = millis();


    Serial.println();
    Serial.println("================================");

    Serial.print("NEW GOAL: ");
    Serial.println(goal_number);

    Serial.print("Current distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    Serial.print("New target: ");
    Serial.print(target);
    Serial.println(" cm");

    Serial.println("================================");
  }


  // ===================================================
  // MOVING
  // ===================================================

  if (!target_reached) {

    // -----------------------------------------------
    // Calculate elapsed time
    // -----------------------------------------------

    unsigned long currentTime = millis();

    float dt = (currentTime - previousTime) / 1000.0;

    previousTime = currentTime;


    // -----------------------------------------------
    // Error to NEXT GOAL
    // -----------------------------------------------

    error = distance - target;


    // -----------------------------------------------
    // Integral
    // -----------------------------------------------

    integral += error * dt;


    // -----------------------------------------------
    // Anti-windup
    // -----------------------------------------------

    integral = constrain(
      integral,
      -INTEGRAL_LIMIT,
      INTEGRAL_LIMIT
    );


    // -----------------------------------------------
    // PI controller
    // -----------------------------------------------

    float control = Kp * error + Ki * integral;


    // -----------------------------------------------
    // Motor velocity
    // -----------------------------------------------

    int velocity = constrain(
      control,
      55,
      MAX_SPEED
    );


    // -----------------------------------------------
    // Set motor speed
    // -----------------------------------------------

    motor1.setSpeed(velocity);
    motor2.setSpeed(velocity);
    motor3.setSpeed(velocity);
    motor4.setSpeed(velocity);


    // -----------------------------------------------
    // Move forward
    // -----------------------------------------------

    forward();


    // -----------------------------------------------
    // Serial monitor
    // -----------------------------------------------

    Serial.print("Distance: ");
    Serial.print(distance);

    Serial.print(" cm | Target: ");
    Serial.print(target);

    Serial.print(" cm | Error: ");
    Serial.print(error);

    Serial.print(" cm | Integral: ");
    Serial.print(integral);

    Serial.print(" | Control: ");
    Serial.print(control);

    Serial.print(" | Velocity: ");
    Serial.println(velocity);


    // =================================================
    // CHECK IF TARGET WAS REACHED
    // =================================================

    if (abs(error) < threshold) {

      stopmotors();

      Serial.println();
      Serial.println("******** TARGET REACHED ********");

      Serial.print("Goal ");
      Serial.print(goal_number);
      Serial.println(" reached.");

      Serial.println("Stopping for 5 seconds...");


      // -----------------------------------------------
      // Stop for 5 seconds
      // -----------------------------------------------

      delay(STOP_TIME);


      // -----------------------------------------------
      // Start next goal
      // -----------------------------------------------

      target_reached = true;

      Serial.println("5 seconds completed.");
      Serial.println("Continuing to next goal...");
    }
  }


  delay(20);
}