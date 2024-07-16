#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

// MPU6050 object
MPU6050 mpu;

// Bluetooth module connected to pins 10 (RX) and 11 (TX)
SoftwareSerial BTSerial(10, 11);

// Motor driver pins
const int dir1 = 3;  // Left motors direction
const int pwm1 = 9;  // Left motors speed control
const int dir2 = 4;  // Right motors direction
const int pwm2 = 10; // Right motors speed control

int speed1 = 0; // Initial speed for left motors
int speed2 = 0; // Initial speed for right motors
const int maxSpeed = 255; // Maximum speed (PWM value)

// Speed increment step
const int speedStep = 10;
const int delayTime = 100;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  pinMode(dir1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm2, OUTPUT);
}

void loop() {
  if (BTSerial.available()) {
    char command = BTSerial.read();
    handleCommand(command);
  }

  detectMovement();
  detectSlipping();

  delay(delayTime);
}

void handleCommand(char command) {
  switch (command) {
    case 'F': accelerateForward(); break;
    case 'B': accelerateBackward(); break;
    case 'L': accelerateTurnLeft(); break;
    case 'R': accelerateTurnRight(); break;
    case 'l': accelerateRotateLeft(); break;
    case 'r': accelerateRotateRight(); break;
    case 's':
    case 'S':stopMovingGradually(); break;
  }
}

void accelerateForward() {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
  increaseSpeed(speed1, maxSpeed, pwm1);
  increaseSpeed(speed2, maxSpeed, pwm2);
  Serial.println("Moving Forward");
}

void accelerateBackward() {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  increaseSpeed(speed1, maxSpeed, pwm1);
  increaseSpeed(speed2, maxSpeed, pwm2);
  Serial.println("Moving Backward");
}

void accelerateTurnLeft() {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  increaseSpeed(speed1, maxSpeed, pwm1);
  increaseSpeed(speed2, maxSpeed, pwm2);
  Serial.println("Turning Left");
}

void accelerateTurnRight() {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  increaseSpeed(speed1, maxSpeed, pwm1);
  increaseSpeed(speed2, maxSpeed, pwm2);
  Serial.println("Turning Right");
}

void accelerateRotateLeft() {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  increaseSpeed(speed1, maxSpeed, pwm1);
  increaseSpeed(speed2, maxSpeed, pwm2);
  Serial.println("Rotating Left");
}

void accelerateRotateRight() {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  increaseSpeed(speed1, maxSpeed, pwm1);
  increaseSpeed(speed2, maxSpeed, pwm2);
  Serial.println("Rotating Right");
}

void stopMovingGradually() {
  decreaseSpeed(speed1, 0, pwm1);
  decreaseSpeed(speed2, 0, pwm2);
  Serial.println("Stopping");
}

void increaseSpeed(int &speed, int targetSpeed, int pwmPin) {
  while (speed < targetSpeed) {
    speed += speedStep;
    analogWrite(pwmPin, speed);
    delay(delayTime);
  }
  analogWrite(pwmPin, targetSpeed);
}

void decreaseSpeed(int &speed, int targetSpeed, int pwmPin) {
  while (speed > targetSpeed) {
    speed -= speedStep;
    analogWrite(pwmPin, speed);
    delay(delayTime);
  }
  analogWrite(pwmPin, targetSpeed);
}

void detectMovement() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  float gyroZ = gz / 131.0;

  if (gyroZ > 50.0) {
    Serial.println("Turning Right");
  } else if (gyroZ < -50.0) {
    Serial.println("Turning Left");
  } else {
    Serial.println("Moving Straight");
  }
}

void detectSlipping() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroZ = gz / 131.0;

  static float prevAccelX = 0.0, prevAccelY = 0.0, prevAccelZ = 0.0;
  float deltaAccelX = abs(accelX - prevAccelX);
  float deltaAccelY = abs(accelY - prevAccelY);
  float deltaAccelZ = abs(accelZ - prevAccelZ);

  if ((deltaAccelX > 0.9 || deltaAccelY > 0.9 || deltaAccelZ > 0.9) && abs(gyroZ) < 50.0) {
    Serial.println("Slipping Detected");
  }

  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;
}
