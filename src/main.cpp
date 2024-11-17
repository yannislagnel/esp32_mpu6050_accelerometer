#include <Arduino.h>
#include <ESP32Servo.h>
// #include <Servo.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the.cpp /.h files
// for both classes must be in the include path of your project
#include <Adafruit_MPU6050.h>  //https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>   //https://github.com/adafruit/Adafruit_Sensor
#include <Wire.h>              //https://www.arduino.cc/en/reference/wire

// Define the pin for the interrupt
#define GYRO_INT 5  // d8
// const int interruptPin = 32;

#define PIN_MOT_L 13  // d7  // Broche de sortie utilisée

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6)
#define PIN_MOT_R 4  // d0

// variable for storing the potentiometer value
// int potValue = 0;

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6)
#define POT_PIN 4

// variable for storing the potentiometer value
int potValue = 0;
char c = 0;
unsigned long currentMillis = 0;
unsigned long startMillis = 0;
//*************************** IMU MPU6050 *******************************
// with the DMP using I2Cdev lib
// MPU6050 mpu; // MPU6050 mpu(0x69); // <-- use for AD0 high
Adafruit_MPU6050 mma;  // = Adafruit_MPU6050();
Servo R;
Servo L;

#define INTERRUPT_PIN GYRO_INT  // use pin 34 for teensy could be any digital

float angle_degres = 0.0;
float elapsedTime, Time, previous_Time;
float rad_to_deg = 180.0 / 3.141592654;

float pid, pid_i, pid_d, pid_p, pwmL, pwmR, error, previous_error;

float previous_pid = 0.0;

/////////////////constantes PID////////////////////////
float kp = 0.1f;
float ki = 0.000f;
float kd = 0.0000f;

float throttle = 1100.0;
float desired_angle = 0;

float angle_degres_1 = 0;
float angle_degres_2 = 0;
float angle_degres_3 = 0;
float angle_degres_4 = 0;
float angle_degres_5 = 0;

float angle_degres_filtre = 0;

// **********************************************************************************
// **********************************************************************************

/* // ISR to handle the interrupt
void IRAM_ATTR handleInterrupt()
{
    delayMicroseconds(5);
    if (digitalRead(interruptPin) == LOW)
        interruptCount++;
} */

// ===  INTERRUPT MPU6050 DETECTION ROUTINE
// volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin
// has gone high void dmpDataReady() { mpuInterrupt = true; }

// void setup_mpu6050(void);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  R.attach(PIN_MOT_R);  // attatch the right motor to pin 3
  L.attach(PIN_MOT_L);  // attatch the left motor to pin 5

  /* R.writeMicroseconds(2000);
  L.writeMicroseconds(2000);

  while (!Serial.available()) {
    delay(1);
  }
  c = Serial.read();
  c = 0; */

  R.writeMicroseconds(1000);
  L.writeMicroseconds(1000);

  while (!Serial.available()) {
    delay(1);
  }
  c = Serial.read();
  c = 0;

  

  // ********* setup MPU6050 ****************************************
  // Try to initialize!
  if (!mma.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mma.setAccelerometerRange(MPU6050_RANGE_2_G);
  // mma.setGyroRange(MPU6050_RANGE_250_DEG);
  mma.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("\nSetup ended .....");

  delay(5000);
  previous_Time = millis() + 1;  // +1 =>to ovoid div by 0
}

void loop() {
  Time = millis();

  c = 0;
  if (Serial.available()) {
    delay(2);  // delay to allow byte to arrive in input buffer
    c = Serial.read();

    if (c == 'P') {
      kp = kp + 0.01;
    }
    if (c == 'p') {
      kp = kp - 0.01;
    }

    if (c == 'I') {
      ki = ki + 0.002;
      pid_i = 0;
    }
    if (c == 'i') {
      ki = ki - 0.002;
      pid_i = 0;
    }

    if (c == 'D') {
      kd = kd + 0.001;
    }
    if (c == 'd') {
      kd = kd - 0.001;
    }
  }

  elapsedTime = (Time - previous_Time) / 1000.0;
  //  if (elapsedTime == 0.0000000)
  //      elapsedTime = 0.0000000001; // to ovoid div by 0

  //************  read MMA 8451 *********************
  ////Read acceleromter data
  sensors_event_t a, g, temp;
  mma.getEvent(&a, &g, &temp);

  angle_degres = atan2(a.acceleration.x, a.acceleration.z) * rad_to_deg;
  // if (abs(angle_degres) < 0.50) {
  //   angle_degres = 0.00;
  // }
  /* angle_degres_filtre = (0.045 * angle_degres + 0.0542 * angle_degres_1 +
                         0.02431 * angle_degres_2 + 0.4008 * angle_degres_3 +
                         0.2431 * angle_degres_4 + 0.0542 * angle_degres_5) /
                        6;

  error = angle_degres_filtre - desired_angle;

  angle_degres_5 = angle_degres_4;
  angle_degres_4 = angle_degres_3;
  angle_degres_3 = angle_degres_2;
  angle_degres_2 = angle_degres_1;
  angle_degres_1 = angle_degres; */

  error = angle_degres - desired_angle;


//  if (abs(error) < 0.50) {
//    error = 0.00;
//  }

  // pid = previous_pid + (kp * error) +
  /*   pid =
        ((kp*error) +(ki * (elapsedTime / 2.0) + (kd / elapsedTime)) * error) +
        (ki * (elapsedTime / 2.0) - (kd / elapsedTime)) * previous_error; */

  pid_p = kp * error;

  //pid_i = (pid_i + error * pid_i) * ki;
  pid_i = (pid_i + error) * ki;

  pid_d = kd * ((error - previous_error) / elapsedTime);

  pid = (pid_p + pid_i + pid_d);

  if (pid < -1000) {
    pid = -1000;
  }
  if (pid > 1000) {
    pid = 1000;
  }

  pwmL = throttle + pid;
  pwmR = throttle - pid;

  if (pwmR < 1000) {
    pwmR = 1000;
  }
  if (pwmR > 2000) {
    pwmR = 2000;
  }
  // Left
  if (pwmL < 1000) {
    pwmL = 1000;
  }
  if (pwmL > 2000) {
    pwmL = 2000;
  }

  L.writeMicroseconds(pwmL);
  R.writeMicroseconds(pwmR);

  Serial.print(">");
  Serial.print(angle_degres);
  Serial.print(",");
  Serial.print(pid);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.print(pwmL);
  Serial.print(",");
  Serial.print(pwmR);
  Serial.print(",");
  Serial.print(desired_angle);
  Serial.print(",");
  Serial.print(kp);
  Serial.print(",");
  Serial.print(ki);
  Serial.print(",");
  Serial.print(kd);
  Serial.print("\n");

  previous_pid = pid;
  previous_error = error;
  previous_Time = Time;
  // delay(10);
}
