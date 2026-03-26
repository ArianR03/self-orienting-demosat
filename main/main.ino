/*
  Self Orienting Payload for Yaw Stabilization
  Spring DemoSat 2026

  THis program intended purpose reads the data from two motor servos and are controlled
  by an internal integrated IMU sensor on the Arduino Nano 33 BLE Sense Rev2. This system
  will be able to rotate a payload autonomously in a 360 degree rotation with minimal error
  during flight. 

*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Arduino_BMI270_BMM150.h>
#include <Servo.h>
#include <cmath>

#define POWER_LED 4
#define SERVO_LED 5
#define PROBLEM_LED 6

// Solar Cell/Photodiode Variables
const int photo1 = A0;
const int photo2 = A1;
const int photo3 = A2;
const int photo4 = A3;
const int solarCell = A6;

// Servo variables
const int servo1PWM = 9;
const int servo2PWM = 10;

// PID Control
double Kp = 2.5;
double Ki = 0.0;
double Kd = 0.0;
double integral;
double previous_error;

// Target
const double targetYaw = 90;

Servo servo1;
Servo servo2;
constexpr double pi = 3.14159;

void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  delay(1000);

  // Set LED params.
  setup_led();

  // Setup input PWM for both Servo motors.
  servo1.attach(servo1PWM);
  servo2.attach(servo2PWM);

  while (!IMU.begin()) {
    Serial.println("Failed to initalize IMU sensor.");
    blink_problem();
  }
  Serial.println("Initalized IMU Sensor.");
  
  digitalWrite(POWER_LED, HIGH);
}

void setup_led(){
  // Set up Power LED
  pinMode(POWER_LED, OUTPUT);
  digitalWrite(POWER_LED, LOW);

  // Setup servo led
  pinMode(SERVO_LED, OUTPUT);
  diigtalWrite(SERVO_LED, LOW);

  // Setup problem led 
  pinMode(PROBLEM_LED, OUTPUT);
  digitalWrite(PROBLEM_LED, LOW);

}

void blink_problem(){
  digitalWrite(PROBLEM_LED, LOW);
  delay(50);
  digitalWrite(PROBLEM_LED, HIGH);
  delay(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  double imuYaw;
  float x, y, z;

  // Read IMU data relative to Earths magnetic field.
  IMU.readMagneticField(x,y,z);
  imuYaw = atan2(y,x) * 180 / pi;
  if (imuYaw < 0) { imuYaw += 360; };

  double err = shortestPath(targetYaw, imuYaw);
  double control = computePID(err);

  int servo1PWMout = 1500 + control;
  int servo2PWMout = 1500 - control;
  servo1.writeMicroseconds(servo1PWMout);
  servo2.writeMicroseconds(servo2PWMout);

  

  delay(20);

}

// Helper Functions
double shortestPath(double target, double curr){
  double shortest_path = fmod((target-curr)+540, 360) - 180; // S - (Target - Current + 540)mod(360) - 180
  return shortest_path;
}

double computePID(double error){
  integral += (error * 0.02);
  double dt = (error - previous_error) / 0.02;
  previous_error = error;

  double out = Kp * error + Ki * integral + Kd * dt;
  
  if (out > 220) return 220;      // Constrain to 220us 1500 -> 1720 limit
  if (out < -220) return -220;    // Constrain to -220us 1500 -> 1280 limit

  // If constraints aren't matched, return normal PID output.
  return out;
}
