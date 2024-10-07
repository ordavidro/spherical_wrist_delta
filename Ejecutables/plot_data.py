#!/usr/bin/env python3 
import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos del archivo CSV
try:
    data = pd.read_csv('imu_and_motor_positions.csv')  # Cambiar a .csv
except Exception as e:
    print(f'Error al leer el archivo CSV: {e}')
    exit()

# Separar los datos por topics
motor_data = data[data['topic'] == '/motor_positions'].copy()  # Usar .copy() para evitar SettingWithCopyWarning
imu_data_pitch = data[data['topic'] == '/roll_angle_topic'].copy()
imu_data_roll = data[data['topic'] == '/pitch_angle_topic'].copy()
imu_data_yaw = data[data['topic'] == '/yaw_angle_topic'].copy()

# Convertir los datos de los ángulos a float
imu_data_roll['data'] = imu_data_roll['data'].astype(float)
imu_data_pitch['data'] = imu_data_pitch['data'].astype(float)
imu_data_yaw['data'] = imu_data_yaw['data'].astype(float)

# Procesar los datos de motor
motor_data[['motor1', 'motor2', 'motor3']] = motor_data['data'].str.split(',', expand=True).astype(float)

# Obtener el primer timestamp y restarlo a toda la columna
first_timestamp = motor_data['timestamp'].values[0]
motor_data['timestamp'] -= first_timestamp
imu_data_roll['timestamp'] -= first_timestamp
imu_data_pitch['timestamp'] -= first_timestamp
imu_data_yaw['timestamp'] -= first_timestamp

# Definir la duración total
duracion = 15  # Duración total en segundos
tiempo_fin = motor_data['timestamp'].values[0] + duracion  # Obtener el primer timestamp

# Filtrar los datos para que solo contengan valores dentro del rango de tiempo
motor_data = motor_data[(motor_data['timestamp'] >= 0) & (motor_data['timestamp'] <= duracion)]
imu_data_roll = imu_data_roll[(imu_data_roll['timestamp'] >= 0) & (imu_data_roll['timestamp'] <= duracion)]
imu_data_pitch = imu_data_pitch[(imu_data_pitch['timestamp'] >= 0) & (imu_data_pitch['timestamp'] <= duracion)]
imu_data_yaw = imu_data_yaw[(imu_data_yaw['timestamp'] >= 0) & (imu_data_yaw['timestamp'] <= duracion)]

# Graficar los datos
plt.figure(figsize=(10, 6)) 

# Graficar los datos de los motores
plt.subplot(2, 1, 1)
plt.plot(motor_data['timestamp'].values, motor_data['motor2'].values, label='Motor 2', color='orange')
plt.plot(motor_data['timestamp'].values, motor_data['motor3'].values, label='Motor 3', color='green')
plt.xlabel('Timestamp (s)')
plt.ylabel('Motor Positions')
plt.title('Motor Positions Over Time')
plt.legend()

# Graficar los datos de la IMU
plt.subplot(2, 1, 2)
plt.plot(imu_data_pitch['timestamp'].values, imu_data_pitch['data'].values, label='Pitch', color='purple')
plt.plot(imu_data_yaw['timestamp'].values, imu_data_yaw['data'].values, label='Yaw', color='brown')
plt.xlabel('Timestamp (s)')
plt.ylabel('IMU Angles (degrees)')
plt.title('IMU Angles Over Time')
plt.legend()

plt.tight_layout()
plt.show()
