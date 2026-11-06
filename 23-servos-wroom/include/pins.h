#pragma once

// ESP32-WROOM-32D del brazo, conexionado del robot (recolector_de_frutas/firmware/brazo/include/pins.h).
// Único lugar con GPIO de esta prueba.

// Servos: señal a 3,3 V; 6 V del buck 3 y GND común con la WROOM.
constexpr int PIN_SERVO_PINZA = 4;   // apertura de la pinza (según el plano)
constexpr int PIN_SERVO_GIRO = 14;   // giro J4 = muñeca (según el plano); pulsa durante el arranque
constexpr int CANAL_PINZA = 0;
constexpr int CANAL_GIRO = 1;

// Los IBT-2 del brazo: EN común y PWM se fijan en LOW para que los motores no se muevan.
constexpr int PIN_BRAZO_EN = 23;
constexpr int PINES_PWM_BRAZO[] = {21, 22, 25, 26, 16, 17};
