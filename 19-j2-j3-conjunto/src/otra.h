#pragma once

#include <stdint.h>

// La articulación que NO se ajusta (J3 si la activa es J2, y al revés): la sostiene sola, en una tarea de FreeRTOS en
// el núcleo 0, a 100 Hz, mientras el resto del firmware (núcleo 1) mueve la activa con sus comandos bloqueantes.
// Usa su propio puente (canales LEDC 2 y 3), su encoder (PCNT 1) y su final de carrera. El EN es común: el firmware
// no tiene que poner el EN en bajo (rueda libre) mientras esta sostiene (ver liberar() en main.cpp).
namespace otra {

bool begin();                 // configura puente, encoder y final de carrera y arranca la tarea (sin sostener)
void cero();                  // cero en la posición actual (lo hace la tarea en su próximo periodo)
void sostener(float grados);  // empieza (o sigue) sosteniendo; la referencia va hacia ese ángulo a OTRA.vel °/s
void parar();                 // deja de sostener: duty 0 (freno); el EN lo maneja quien llama
bool activa();
float grados();               // ángulo medido
float destino();              // ángulo pedido (al que va o en el que está)
float duty();                 // % aplicado
bool finalCarrera();          // apretado
void ganancias(float khp, float khi);
float khp();
float khi();
// Aviso pendiente de la tarea (no imprime por Serial desde el núcleo 0): devuelve nullptr si no hay
const char *aviso();

}  // namespace otra
