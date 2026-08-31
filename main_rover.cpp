#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>
#define MPU6050 0x68

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
unsigned long previousTime;
float dt;


const int TRIG = A0;
const int ECHO = A1;
const float Kp = 4.0;
const float Kp2 = 30.0;
const int MAX_SPEED = 255;
const int MIN_SPEED = 75;
float delta_v;


// M1 is forward left
// M2 is backward left
// M3 is forward right
// M4 is backward right

AF_DCMotor fl(1);
AF_DCMotor bl(2);
AF_DCMotor fr(3);
AF_DCMotor br(4);

void setup() {

  // Initial  motor speed
  fl.setSpeed(200);
  bl.setSpeed(200);
  fr.setSpeed(200);
  br.setSpeed(200);

  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);

  Wire.begin();
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  previousTime = micros();

  // 20 second delay to protet your computer
  delay(20000);
}

void set_speed(float ref, float delta_v){
  fl.setSpeed(ref);
  bl.setSpeed(ref);
  fr.setSpeed(ref + delta_v);
  br.setSpeed(ref + delta_v);
}


void rotate(){
  fl.run(FORWARD);
  bl.run(BACKWARD);
  fr.run(FORWARD);
  br.run(BACKWARD);
  
}


void forward(){
  fl.run(BACKWARD);
  bl.run(FORWARD);
  fr.run(FORWARD);
  br.run(BACKWARD);
}


void stopmotors(){
  fl.run(RELEASE);
  bl.run(RELEASE);
  fr.run(RELEASE);
  br.run(RELEASE);
}


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

  float distance = duration * 0.0343 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}


float read_gyroscope(){
  unsigned long currentTime = micros();
  dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  int16_t rawAx, rawAy, rawAz;
  int16_t rawGx, rawGy, rawGz;

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 14);
  rawAx = Wire.read() << 8 | Wire.read();
  rawAy = Wire.read() << 8 | Wire.read();
  rawAz = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();  
  rawGx = Wire.read() << 8 | Wire.read();
  rawGy = Wire.read() << 8 | Wire.read();
  rawGz = Wire.read() << 8 | Wire.read();

  ax = (rawAx / 16384.0) * 9.81;
  ay = (rawAy / 16384.0) * 9.81;
  az = (rawAz / 16384.0) * 9.81;
  az = az - 9.81;
  gx = rawGx / 131.0;
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;
  vx = vx + ax * dt;
  vy = vy + ay * dt;
  vz = vz + az * dt;
  px = px + vx * dt;
  py = py + vy * dt;
  pz = pz + vz * dt;
  roll  = roll  + gx * dt;
  pitch = pitch + gy * dt;
  yaw   = yaw   + gz * dt;

  return yaw;
}


void loop() {

  float distance = getDistance();
  yaw = read_gyroscope();
  delta_v = yaw*Kp2;
  delta_v = constrain(delta_v, -100, 100);
  set_speed(150,delta_v);
  forward();
  
}