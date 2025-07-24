#!/bin/bash
clear
echo "🚀 Starting the simulator container..."
xhost +local:root
docker compose down --remove-orphans
docker compose up

