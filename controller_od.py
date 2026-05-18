#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
import numpy as np


class ControllerOd(Node):
    def __init__(self):
        super().__init__('controller_od')

        # === MODO RECTA (rapido) ===
        self.kp_low_recta = 0.5
        self.kp_high_recta = 3.2
        self.v_recta = 0.32
        self.max_w_recta = 2.0

        # === MODO CURVA (conservador, lo que funcionaba antes) ===
        self.kp_low_curva = 0.5
        self.kp_high_curva = 1.8
        self.v_curva = 0.11
        self.max_w_curva = 1.5

        # Umbral para entrar a modo curva
        self.curva_threshold = 0.3        # Tiempo minimo en modo curva
        self.tiempo_curva = 0.5
        # Umbral para considerar error "normalizado"
        self.error_normal = 0.15

        # Topes generales
        self.max_v = 0.35

        self.dt = 0.02
        self.error = 0.0
        self.line_detected = False
        self.tiempo_restante = 0.0
        self.traffic_state = "NORMAL"

        self.create_subscription(Float32, '/line_error', self.error_cb, 10)
        self.create_subscription(Bool, '/line_detected', self.det_cb, 10)
        self.create_subscription(String, '/detected_color', self.color_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.loop)
        self.get_logger().info("Controlador P listo")

    def error_cb(self, msg):
        self.error = msg.data

    def det_cb(self, msg):
        self.line_detected = msg.data

    def color_cb(self, msg):
        color = msg.data
        old = self.traffic_state
        if color == "none":
            return
        if self.traffic_state == "NORMAL":
            if color == "yellow":
                self.traffic_state = "SLOW"
            elif color == "red":
                self.traffic_state = "STOPPED"
        elif self.traffic_state == "SLOW":
            if color == "red":
                self.traffic_state = "STOPPED"
        elif self.traffic_state == "STOPPED":
            if color == "green":
                self.traffic_state = "NORMAL"
        if old != self.traffic_state:
            self.get_logger().info(f"Semaforo: {old} -> {self.traffic_state}")

    def get_speed_factor(self):
        return 1.0

    def loop(self):
        cmd = Twist()
        if not self.line_detected:
            self.cmd_pub.publish(cmd)
            return

        err_abs = abs(self.error)

        # Si detectamos curva, ponemos timer a 3 segundos
        if err_abs >= self.curva_threshold:
            self.tiempo_restante = self.tiempo_curva
        else:
            # Solo bajamos el timer si el error YA esta normalizado
            if err_abs < self.error_normal:
                self.tiempo_restante -= self.dt
                if self.tiempo_restante < 0:
                    self.tiempo_restante = 0
            # Si error esta entre 0.1 y 0.3, mantiene timer (no baja)

        # Elegimos ganancias y velocidad segun modo
        if self.tiempo_restante > 0:
            # Modo curva (conservador, el que sí funciona)
            kp_low = self.kp_low_curva
            kp_high = self.kp_high_curva
            v = self.v_curva
            max_w = self.max_w_curva
        else:
            # Modo recta (rapido)
            kp_low = self.kp_low_recta
            kp_high = self.kp_high_recta
            v = self.v_recta
            max_w = self.max_w_recta

        # Ganancia gradual
        kp = kp_low + (kp_high - kp_low) * err_abs
        w = -kp * self.error
        w = np.clip(w, -max_w, max_w)

        v = np.clip(v, 0.0, self.max_v)

        factor = self.get_speed_factor()
        v *= factor

        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerOd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
