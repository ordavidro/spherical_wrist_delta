#!/bin/bash

# Nombre del nodo y nombre del archivo rosbag
NODE_NAME="ControlTopic"
BAG_FILE="ControlTopic.bag"

# Tópicos a grabar
TOPICS=("/user_position_input" "/motor_positions")

# Función para verificar si un nodo está corriendo
function is_node_running {
    if rosnode list | grep -q "$NODE_NAME"; then
        return 0
    else
        return 1
    fi
}

# Ejecutar el nodo ControlTopic
echo "Starting ControlTopic node..."
rosrun wrist_control ControlTopic &

# Esperar a que el nodo se inicie
echo "Waiting for ControlTopic node to start..."
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

# Capturar el PID del proceso ControlTopic para poder terminarlo más tarde
CONTROL_TOPIC_PID=$!

# Capturar el PID del proceso de rosbag
ROSBAG_PID=$!

# Manejar la señal de terminación (Ctrl+C)
trap "echo 'Terminating rosbag recording and ControlTopic node...'; kill $ROSBAG_PID $CONTROL_TOPIC_PID; exit 0" SIGINT SIGTERM

# Esperar indefinidamente
wait $ROSBAG_PID
