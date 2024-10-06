#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos del archivo Excel
try:
    data = pd.read_excel('imu_and_motor_positions.xlsx', engine='openpyxl')  # Asegúrate de que la extensión sea .xlsx
except Exception as e:
    print(f'Error al leer el archivo Excel: {e}')
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

# Definir la duración total
duracion = 15  # Duración total en segundos
tiempo_inicio = motor_data['timestamp'].values[0]  # Obtener el primer timestamp

# Calcular el tiempo de fin
tiempo_fin = tiempo_inicio + duracion

# Filtrar los datos para que solo contengan valores dentro del rango de tiempo
motor_data = motor_data[(motor_data['timestamp'] >= tiempo_inicio) & (motor_data['timestamp'] <= tiempo_fin)]
imu_data_roll = imu_data_roll[(imu_data_roll['timestamp'] >= tiempo_inicio) & (imu_data_roll['timestamp'] <= tiempo_fin)]
imu_data_pitch = imu_data_pitch[(imu_data_pitch['timestamp'] >= tiempo_inicio) & (imu_data_pitch['timestamp'] <= tiempo_fin)]
imu_data_yaw = imu_data_yaw[(imu_data_yaw['timestamp'] >= tiempo_inicio) & (imu_data_yaw['timestamp'] <= tiempo_fin)]

# Graficar los datos
plt.figure(figsize=(10, 6))

# Graficar los datos de los motores
plt.subplot(2, 1, 1)
#plt.plot(motor_data['timestamp'].values, motor_data['motor1'].values, label='Motor 1', color='blue')
plt.plot(motor_data['timestamp'].values, motor_data['motor2'].values, label='Motor 2', color='orange')
plt.plot(motor_data['timestamp'].values, motor_data['motor3'].values, label='Motor 3', color='green')
plt.xlabel('Timestamp (s)')
plt.ylabel('Motor Positions')
plt.title('Motor Positions Over Time')
plt.legend()

# Graficar los datos de la IMU
plt.subplot(2, 1, 2)
#plt.plot(imu_data_roll['timestamp'].values, imu_data_roll['data'].values, label='Roll', color='red')
plt.plot(imu_data_pitch['timestamp'].values, imu_data_pitch['data'].values, label='Pitch', color='purple')
plt.plot(imu_data_yaw['timestamp'].values, imu_data_yaw['data'].values, label='Yaw', color='brown')
plt.xlabel('Timestamp (s)')
plt.ylabel('IMU Angles (degrees)')
plt.title('IMU Angles Over Time')
plt.legend()

plt.tight_layout()
plt.show()
