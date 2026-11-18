#!/usr/bin/env python3
"""25: pinza y muñeca lo más rápido posible (demostración), por /brazo/comando.

Corre en la Pi con el agente del brazo (lanzar.sh). Pone la rampa de los dos servos en 0 (salto directo: van a la
velocidad propia del servo, MG996R ~0,2 s / 60° a 6 V) y repite: abrir pinza -> muñeca a un lado -> al otro ->
cerrar pinza. Al terminar (o con Ctrl+C) deja la muñeca al centro y la rampa de vuelta en 45 °/s (la de siempre).

No mueve J1-J3. Manos lejos de la pinza.
Uso: python3 servos_rapido.py [vueltas (3)] [--espera s (0.5)]
"""
import argparse
import time

import rclpy
from std_msgs.msg import String

W_NORMAL = 45  # °/s, SERVO_W_INICIAL de 20-brazo-coordinado


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('vueltas', type=int, nargs='?', default=3)
    ap.add_argument('--espera', type=float, default=0.5, help='s entre comandos (lo que tarda el servo en llegar)')
    a = ap.parse_args()

    rclpy.init()
    nodo = rclpy.create_node('servos_rapido')
    pub = nodo.create_publisher(String, '/brazo/comando', 10)
    time.sleep(1.0)  # que el publicador se descubra con el agente

    def cmd(texto, espera=0.0):
        pub.publish(String(data=texto))
        print(texto)
        if espera:
            time.sleep(espera)

    try:
        cmd('pz w 0', 0.05)
        cmd('mn w 0', 0.05)
        for k in range(a.vueltas):
            print(f'--- vuelta {k + 1} de {a.vueltas}')
            cmd('pz a', a.espera)   # abrir
            cmd('mn a', a.espera)   # muñeca 90° horario
            cmd('mn b', a.espera)   # 90° antihorario (180°: tarda más)
            cmd('mn 0', a.espera)   # centro
            cmd('pz c', a.espera)   # cerrar
    except KeyboardInterrupt:
        pass
    finally:
        cmd('mn 0', 0.3)
        cmd(f'pz w {W_NORMAL}', 0.05)
        cmd(f'mn w {W_NORMAL}', 0.2)
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
