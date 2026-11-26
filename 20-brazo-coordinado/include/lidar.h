#pragma once
void lidarBegin();
void lidarLoop();
// Metros; NaN si no hay una lectura válida de los últimos 300 ms.
float lidarMetros();
