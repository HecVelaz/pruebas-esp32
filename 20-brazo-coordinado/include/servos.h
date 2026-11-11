#pragma once
// Pinza y muñeca de 20 (comandos "pz" y "mn", como "p" y "m" de 23). Se atienden en loop(), fuera de la tarea de
// control: no cortan un mv ni el homing.
bool servosIniciar();
bool servosComando(const char *linea);  // true si era "pz ..." o "mn ..."
void servosActualizar();                 // llamar seguido desde loop(): rampa y aviso de llegada
