#include "Wire.h"
#include <MPU6050_light.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include <ros.h>
#include <std_msgs/Float32.h>

MPU6050 mpu(Wire);

ros::NodeHandle nh_accelerometer;

std_msgs::Float32 roll_msg;
std_msgs::Float32 yaw_msg;
std_msgs::Float32 pitch_msg;
ros::Publisher roll_pub("roll_angle_topic", &roll_msg);
ros::Publisher yaw_pub("yaw_angle_topic", &yaw_msg);
ros::Publisher pitch_pub("pitch_angle_topic", &pitch_msg);

long timer = 0;

// Offsets for calibration
float roll_offset = -230;   // Adjusted offset for roll
float yaw_offset = 100;   // Adjusted offset for yaw

// Variables to store the last published angles
float last_roll_angle = 0;
float last_yaw_angle = 0;
float last_pitch_angle = 0;

// Function to normalize angles between 0 and 360 degrees
float normalizeAngle(float angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  nh_accelerometer.initNode();
  nh_accelerometer.advertise(roll_pub);
  nh_accelerometer.advertise(yaw_pub);
  nh_accelerometer.advertise(pitch_pub);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    // stop everything if could not connect to MPU6050
    delay(1000);
  }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();

//  if (millis() - timer > 50) { // check data every second
    // Get and normalize angles with offsets
    float roll_angle = normalizeAngle(mpu.getAngleX() + roll_offset);
    float yaw_angle = normalizeAngle(mpu.getAngleY() + yaw_offset);
    float pitch_angle = normalizeAngle(mpu.getAngleZ());

    // Check if the variation in angles is greater than 10 degrees
    if (abs(roll_angle - last_roll_angle) > 1) {
      roll_msg.data = roll_angle;
      roll_pub.publish(&roll_msg);
      last_roll_angle = roll_angle;
    }
    
    if (abs(yaw_angle - last_yaw_angle) > 1) {
      yaw_msg.data = yaw_angle;
      yaw_pub.publish(&yaw_msg);
      last_yaw_angle = yaw_angle;
    }
    
    if (abs(pitch_angle - last_pitch_angle) > 1) {
      pitch_msg.data = pitch_angle;
      pitch_pub.publish(&pitch_msg);
      last_pitch_angle = pitch_angle;
    }
/*
    // Serial output for debugging
    Serial.print(F("TEMPERATURE: ")); Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: ")); Serial.print(mpu.getAccX());
    Serial.print("\tY: "); Serial.print(mpu.getAccY());
    Serial.print("\tZ: "); Serial.println(mpu.getAccZ());

    Serial.print(F("GYRO      X: ")); Serial.print(mpu.getGyroX());
    Serial.print("\tY: "); Serial.print(mpu.getGyroY());
    Serial.print("\tZ: "); Serial.println(mpu.getGyroZ());

    Serial.print(F("ACC ANGLE X: ")); Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: "); Serial.println(mpu.getAccAngleY());

    Serial.print(F("ANGLE     X: ")); Serial.print(roll_angle);
    Serial.print("\tY: "); Serial.print(yaw_angle);
    Serial.print("\tZ: "); Serial.println(pitch_angle);
    Serial.println(F("=====================================================\n"));
*/
 //   timer = millis();
 // }

  nh_accelerometer.spinOnce();
  delay(10); // Delay to control loop rate
}
