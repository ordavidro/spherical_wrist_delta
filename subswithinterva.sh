#!/bin/bash

INTERVAL=5

while true; do
    for TOPIC in "$@"; do
        echo "Topic: $TOPIC"
        rostopic echo -n 1 $TOPIC
    done
    sleep $INTERVAL
done
