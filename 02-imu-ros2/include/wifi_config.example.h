#pragma once

// Copiar este archivo como include/wifi_config.h (está en .gitignore) y completar los datos.
// Solo se usa al compilar el entorno wifi: pio run -e wifi -t upload

#define WIFI_SSID "mi-red"
#define WIFI_PASSWORD "mi-clave"

// IP de la PC donde corre el micro-ROS Agent (udp4) y su puerto
#define AGENT_IP 192, 168, 1, 100
#define AGENT_PORT 8888
