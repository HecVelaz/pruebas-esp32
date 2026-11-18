#!/usr/bin/env python3
"""Muestra en vivo el roll y el pitch de la base (misma cuenta que el firmware) y la seguridad, para probar el vuelco.

En la Pi, con el agente de la base. El firmware marca VUELCO con |roll| o |pitch| >= 25° (base_config.h, filtro de
~0,2 s) y queda enclavado hasta "r" con la base nivelada. Ctrl+C sale y muestra el máximo alcanzado.

Uso:
    python3 ver_inclinacion.py
"""
import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

INCL_MAX = 25.0


def main():
    rclpy.init()
    n = rclpy.create_node("ver_inclinacion")
    d = {"roll": math.nan, "pitch": math.nan, "estado": "", "max_r": 0.0, "max_p": 0.0}

    def al_imu(m):
        a = m.linear_acceleration
        r = math.degrees(math.atan2(a.y, a.z))
        p = math.degrees(math.atan2(-a.x, math.hypot(a.y, a.z)))
        d["roll"], d["pitch"] = r, p
        d["max_r"] = max(d["max_r"], abs(r))
        d["max_p"] = max(d["max_p"], abs(p))

    def al_estado(m):
        d["estado"] = m.data

    n.create_subscription(Imu, "/imu/data_raw", al_imu, qos_profile_sensor_data)
    n.create_subscription(String, "/base/estado", al_estado, 10)

    def mostrar():
        e = d["estado"]
        seg = e.split("seguridad=")[1].split(" imu=")[0] if "seguridad=" in e else "?"
        r, p = d["roll"], d["pitch"]
        marca = "  <-- pasa 25°" if max(abs(r), abs(p)) >= INCL_MAX else ""
        print(f"roll {r:+6.1f}°  pitch {p:+6.1f}°  | seguridad: {seg}{marca}", flush=True)

    n.create_timer(0.25, mostrar)
    print("Inclinar la base: VUELCO con |roll| o |pitch| >= 25°. Ctrl+C sale.")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    print(f"\nMáximo: |roll| {d['max_r']:.1f}°, |pitch| {d['max_p']:.1f}°")
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
