#pragma once

#include <stdint.h>

// La articulación que NO se ajusta (J3 si la activa es J2, y al revés): la sostiene sola, en una tarea de FreeRTOS en
// el núcleo 0, a 100 Hz, mientras el resto del firmware (núcleo 1) mueve la activa con sus comandos bloqueantes.
// Usa su propio puente (canales LEDC 2 y 3), su encoder (PCNT 1) y su final de carrera. El EN es común: el firmware
// no tiene que poner el EN en bajo (rueda libre) mientras esta sostiene (ver liberar() en main.cpp).
namespace otra {

bool begin();                 // configura puente, encoder y final de carrera y arranca la tarea (sin sostener)
void cero();                  // cero en la posición actual (lo hace la tarea en su próximo periodo)
void sostener(float grados);  // empieza (o sigue) sosteniendo; la referencia va hacia ese ángulo a OTRA.vel °/s.
                              // También destraba un ATASCO
void parar();                 // deja de sostener: duty 0 (freno, lo aplica la tarea); el EN lo maneja quien llama
bool activa();
float grados();               // ángulo medido
float destino();              // ángulo pedido (al que va o en el que está)
float referencia();           // referencia del lazo: va del ángulo de partida al destino a OTRA.vel °/s
bool enCamino();              // sosteniendo y todavía no llegó (ver llego())
// Llegó: la referencia en el destino, |destino - medido| < TOL_POS y quieta (< 1 °/s) durante LLEGADA_MS (los de su
// config, como la activa); deja de haber llegado si se aleja más de TOL_SALIDA o cambia el destino.
bool llego();
uint32_t msDesdeLlegada();    // 0 si no llegó
bool frenada();               // llegó y el freno dentro de la banda está prendido: duty 0
float uHold();                // duty aprendido (%): con el modelo (fase 3b), la diferencia con el modelo
// Banda del integrador (°) y freno dentro de la banda (ver config.h)
void opciones(float bandaInt, bool frenoEnBanda);
// Fase 4: homing de la otra. homingIniciar(): si está apretada se aleja hasta soltarlo y HOMING_SALIDA_GRADOS más,
// después baja a HOMING_VEL_BUSCAR hasta que se aprieta (flanco) y se queda ahí sosteniendo. La tarea lo hace.
enum class Homing : uint8_t { Nada, EnCurso, Listo, Error };
void homingIniciar();
Homing homingEstado();
const char *homingError();
void homingCancelar();
float flancoGrados();         // dónde quedó el flanco, con el cero que había
void ceroEnFlanco();          // pone el cero en el flanco (lo hace la tarea; vuelve cuando está hecho)
// Fase 3b: ángulo de la activa (lo publica main.cpp; el modelo del sostener depende de q2 y q3) y modelo sí/no
void anguloActiva(float grados);
void modelo(bool usar);
bool modelo();
float duty0();                // término del modelo que suma ahora (%)
// Compensación de fricción al sostener (%, ver FR_SOSTENER en config.h)
void friccion(float pct);
float friccion();
// Frecuencia del PWM: la cambia la tarea en su próximo periodo (pasando por duty 0). pwmHz(): la vigente
void pwm(uint32_t hz);
uint32_t pwmHz();
float duty();                 // % aplicado
bool finalCarrera();          // apretado
void ganancias(float khp, float khi);
float khp();
float khi();
// Aviso pendiente de la tarea (no imprime por Serial desde el núcleo 0): devuelve nullptr si no hay
const char *aviso();

}  // namespace otra
