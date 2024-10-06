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

// Offsets for calibration
float roll_offset = -230;   // Ajuste para el ángulo de roll
float yaw_offset = 100;     // Ajuste para el ángulo de yaw

// Variables para almacenar los últimos ángulos publicados
float last_roll_angle = 0;
float last_yaw_angle = 0;
float last_pitch_angle = 0;

// Función para normalizar ángulos entre 0 y 360 grados
float normalizeAngle(float angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Inicializa los nodos ROS
  nh_accelerometer.initNode();
  nh_accelerometer.advertise(roll_pub);
  nh_accelerometer.advertise(yaw_pub);
  nh_accelerometer.advertise(pitch_pub);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    // Detener si no se puede conectar al MPU6050
    delay(1000);
  }

  Serial.println(F("Calculando offsets, no mueva el MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // Calibrar giroscopio y acelerómetro
  Serial.println("¡Listo!\n");
}

void loop() {
  mpu.update();

  // Obtener y normalizar los ángulos con los offsets
  float roll_angle = normalizeAngle(mpu.getAngleX() + roll_offset);
  float yaw_angle = normalizeAngle(mpu.getAngleY() + yaw_offset);
  float pitch_angle = normalizeAngle(mpu.getAngleZ());

  // Publicar nuevos valores si la variación es mayor a 1 grado
  if (abs(roll_angle - last_roll_angle) > 2) {
    roll_msg.data = roll_angle;
    roll_pub.publish(&roll_msg);
    last_roll_angle = roll_angle;
  }

  if (abs(yaw_angle - last_yaw_angle) > 2) {
    yaw_msg.data = yaw_angle;
    yaw_pub.publish(&yaw_msg);
    last_yaw_angle = yaw_angle;
  }

  if (abs(pitch_angle - last_pitch_angle) > 2) {
    pitch_msg.data = pitch_angle;
    pitch_pub.publish(&pitch_msg);
    last_pitch_angle = pitch_angle;
  }

  // Ejecutar el ciclo de ROS
  nh_accelerometer.spinOnce();
  delay(100); // Retraso para controlar la frecuencia del bucle
}
