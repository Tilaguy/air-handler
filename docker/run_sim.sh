#!/bin/bash
clear
echo "🚀 Iniciando el contenedor del simulador..."
xhost +local:root
docker compose down --remove-orphans
docker compose up

