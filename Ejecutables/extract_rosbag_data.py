#!/usr/bin/env python3

import rosbag
import csv

bag_file = 'imu_and_motor_positions2.bag'
csv_file = 'imu_and_motor_positions2.csv'

topics = ['/roll_angle_topic', '/pitch_angle_topic', '/yaw_angle_topic', '/motor_positions']

with rosbag.Bag(bag_file) as bag:
    with open(csv_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'topic', 'data'])  # Encabezados CSV

        for topic, msg, t in bag.read_messages(topics=topics):
            if topic in ['/roll_angle_topic', '/pitch_angle_topic', '/yaw_angle_topic']:
                data = msg.data
            elif topic == '/motor_positions':
                data = ','.join(map(str, msg.data))  # Convertir la lista de posiciones a una cadena separada por comas
            writer.writerow([t.to_sec(), topic, data])
