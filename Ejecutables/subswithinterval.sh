#!/bin/bash

INTERVAL=0

while true; do
    # Definir los tópicos a leer
    TOPICS=("/pitch_angle_topic" "/roll_angle_topic" "/yaw_angle_topic")

    # Crear un array para almacenar los valores
    VALUES=()

    for TOPIC in "${TOPICS[@]}"; do
        echo "Reading from topic: $TOPIC"

        # Usar rostopic echo y formatear la salida
        VALUE=$(rostopic echo -n 1 $TOPIC | awk '/data:/ {print $2}')
        
        # Manejo de valores vacíos
        if [ -z "$VALUE" ]; then
            VALUE="null"  # O cualquier otro valor que desees mostrar en caso de error
        fi

        VALUES+=("$VALUE") # Almacenar el valor leído en el array
    done

    # Imprimir los valores en el formato [_, _, _]
    echo "[${VALUES[0]}, ${VALUES[1]}, ${VALUES[2]}]"

    sleep $INTERVAL
done
