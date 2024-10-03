#!/bin/bash

# Nombre del nodo y nombre del archivo rosbag
NODE_NAME="wrist_orientation_node"
BAG_FILE="imu_and_motor_positions2.bag"

# Tópicos a grabar
TOPICS=("/roll_angle_topic" "/pitch_angle_topic" "/yaw_angle_topic" "/motor_positions")

# Función para verificar si un nodo está corriendo
function is_node_running {
    if rosnode list | grep -q "$NODE_NAME"; then
        return 0
    else
        return 1
    fi
}

# Ejecutar el nodo ControlImu
echo "Starting ControlImu node..."
rosrun wrist_control ControlImu &

# Esperar a que el nodo se inicie
echo "Waiting for ControlImu node to start..."
sleep 5  # Ajusta este tiempo según sea necesario

# Verificar si el nodo está corriendo
if ! is_node_running; then
    echo "Error: $NODE_NAME did not start correctly."
    exit 1
fi

# Iniciar la grabación con rosbag
echo "Starting rosbag recording..."
rosbag record -O $BAG_FILE ${TOPICS[@]} &

# Capturar el PID del proceso de rosbag
ROSBAG_PID=$!

# Capturar el PID del proceso ControlImu para poder terminarlo más tarde
CONTROL_IMU_PID=$!

# Capturar el PID del proceso de rosbag
ROSBAG_PID=$!

# Manejar la señal de terminación (Ctrl+C)
trap "echo 'Terminating rosbag recording and ControlImu node...'; kill $ROSBAG_PID $CONTROL_IMU_PID; exit 0" SIGINT SIGTERM

# Esperar indefinidamente
wait $ROSBAG_PID
