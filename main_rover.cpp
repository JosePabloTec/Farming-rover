#include <AFMotor.h>


// M1 es delantera izquierda
// M2 es la trasera izquierda
// M3 es la delnatera derecha
// M4 es la trasera derecha


AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

void setup() {

  // Velocidad de los motores: 0 - 255
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);

  // Espera inicial de 10 segundos
  delay(10000);
}


void rotate(){
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  
}


void forward(){
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}


void stopmotors(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}


void loop() {

  forward();
  delay(1000);

  stopmotors();
  delay(1000);

  rotate();
  delay(1000);

  stopmotors();
  delay(1000);
  
}