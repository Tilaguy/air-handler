#!/bin/bash
echo "🚀 Iniciando el contenedor del simulador..."
docker compose down --remove-orphans
docker compose up

