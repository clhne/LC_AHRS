#include"Wire.h"
#include"I2Cdev.h"
#include"MPU6050.h"
#include "Kalman.h"
Kalman kalmanX;
Kalman kalmanY;

#define ExtResetPin 23

MPU6050 mpu;
float imu[6];
int16_t ax, ay, az, gx, gy, gz;
float roll, pitch;
double roll_kalman, pitch_kalman;
long int timer;

float gx_offset = -502.42;
float gy_offset = 248.72;
float gz_offset = 15.75;
float ax_offset = 140.79;
float ay_offset = 122.76;
float az_offset = 1219.97;
float gyro_sen = 131.0;
float acc_sen = 16384.0;

void setup() {
  pinMode(ExtResetPin, OUTPUT);
  digitalWrite(ExtResetPin, LOW);
  delay(1000);
  digitalWrite(ExtResetPin, HIGH);
  
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  mpu.setRate(400);
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  imu[0] = (ax - ax_offset) / acc_sen;
  imu[1] = (ay - ay_offset) / acc_sen;
  imu[2] = (az - az_offset) / acc_sen;
  roll  = -atan2(imu[1], imu[2]) * RAD_TO_DEG;
  pitch = atan(imu[0] / sqrt(imu[1] * imu[1] + imu[2] * imu[2])) * RAD_TO_DEG;
  Serial.print(roll); Serial.print("\t");
  Serial.println(pitch);
  Serial.println("=============Initial guesses=============");

  // starting angle setting
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  timer = micros();
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  imu[0] = (ax - ax_offset) / acc_sen;
  imu[1] = (ay - ay_offset) / acc_sen;
  imu[2] = (az - az_offset) / acc_sen;
  imu[3] = (gx - gx_offset) / gyro_sen;
  imu[4] = (gy - gy_offset) / gyro_sen;
  imu[5] = (gz - gz_offset) / gyro_sen;

  double dt = (double)(micros() - timer) / 1000000;
  //Serial.println(dt, 4);
  timer = micros();
  
  roll  = -atan2(imu[1], imu[2]) * RAD_TO_DEG;
  pitch = atan(imu[0] / sqrt(imu[1] * imu[1] + imu[2] * imu[2])) * RAD_TO_DEG;

  roll_kalman = kalmanX.getAngle(roll, imu[3], dt);
  pitch_kalman = kalmanY.getAngle(pitch, imu[4], dt);

  /*for(int i = 0; i < 6; i ++){
    Serial.print(imu[i]);   Serial.print(", ");
  }*/

  Serial.print("Kalman(R/P):\t");
  Serial.print(roll_kalman); Serial.print("\t");
  Serial.println(pitch_kalman);

}
