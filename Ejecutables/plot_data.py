import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos del CSV
data = pd.read_csv('imu_and_motor_positions2.csv')

# Verificar los nombres de las columnas
print(data.columns)  # Esto imprimirá los nombres de las columnas en el DataFrame

# Separar los datos por topics
motor_data = data[data['topic'] == '/motor_positions'].copy()  # Usar .copy() para evitar SettingWithCopyWarning
imu_data_roll = data[data['topic'] == '/roll_angle_topic'].copy()
imu_data_pitch = data[data['topic'] == '/pitch_angle_topic'].copy()
imu_data_yaw = data[data['topic'] == '/yaw_angle_topic'].copy()

# Convertir la columna 'data' a tipo string
motor_data.loc[:, 'data'] = motor_data['data'].astype(str)

# Dividir la columna 'data' en tres columnas: 'motor1', 'motor2', 'motor3'
motor_data[['motor1', 'motor2', 'motor3']] = motor_data['data'].str.split(',', expand=True).astype(float)

# Convertir los datos de los ángulos a float
imu_data_roll.loc[:, 'data'] = imu_data_roll['data'].astype(float)
imu_data_pitch.loc[:, 'data'] = imu_data_pitch['data'].astype(float)
imu_data_yaw.loc[:, 'data'] = imu_data_yaw['data'].astype(float)

# Graficar los datos
plt.figure(figsize=(10, 6))

# Graficar los datos de los motores
plt.subplot(2, 1, 1)
plt.plot(motor_data['timestamp'].values, motor_data['motor1'].values, label='Motor 1', color='blue')
plt.plot(motor_data['timestamp'].values, motor_data['motor2'].values, label='Motor 2', color='orange')
plt.plot(motor_data['timestamp'].values, motor_data['motor3'].values, label='Motor 3', color='green')
plt.xlabel('Timestamp (s)')
plt.ylabel('Motor Positions')
plt.title('Motor Positions Over Time')
plt.legend()

# Graficar los datos de la IMU
plt.subplot(2, 1, 2)
plt.plot(imu_data_roll['timestamp'].values, imu_data_roll['data'].values, label='Roll', color='red')
plt.plot(imu_data_pitch['timestamp'].values, imu_data_pitch['data'].values, label='Pitch', color='purple')
plt.plot(imu_data_yaw['timestamp'].values, imu_data_yaw['data'].values, label='Yaw', color='brown')
plt.xlabel('Timestamp (s)')
plt.ylabel('IMU Angles (degrees)')
plt.title('IMU Angles Over Time')
plt.legend()

plt.tight_layout()
plt.show()
