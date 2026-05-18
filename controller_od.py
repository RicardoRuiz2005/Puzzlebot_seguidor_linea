#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
import numpy as np
class ControllerOd(Node):
    def __init__(self):
        super().__init__('controller_od')
        # Ganancias adaptativas:
        # kp_low para recta (no oscilar con imperfecciones)
        # kp_high para curva (girar fuerte)
        # La transicion entre ambas es gradual segun el tamaño del error
        self.kp_low = 0.5
        self.kp_high = 1.8
        # Velocidad base, arrancamos seguro y subimos despues
        self.v_base = 0.1
        # Topes del robot real (sacados de la otra actividad)
        self.max_v = 0.2
        self.max_w = 1.5
        # Cada cuanto mandamos velocidad
        self.dt = 0.02
        # Variables del control
        self.error = 0.0
        self.line_detected = False
        # Estado del semaforo
        self.traffic_state = "NORMAL"
        # Suscripciones
        self.create_subscription(Float32, '/line_error', self.error_cb, 10)
        self.create_subscription(Bool, '/line_detected', self.det_cb, 10)
        self.create_subscription(String, '/detected_color', self.color_cb, 10)
        # Publicamos velocidad
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
        # DESACTIVADO de momento para probar solo el seguidor
        # El detector sigue corriendo pero no afecta la velocidad
        return 1.0
        # Cuando lo reactivemos:
        # if self.traffic_state == "STOPPED":
        #     return 0.0
        # elif self.traffic_state == "SLOW":
        #     return 0.5
        # return 1.0
    def loop(self):
        cmd = Twist()
        # Si no vemos la linea nos quedamos quietos
        if not self.line_detected:
            self.cmd_pub.publish(cmd)
            return
        # Ganancia gradual segun el tamaño del error
        # error chico -> kp_low (suave), error grande -> kp_high (fuerte)
        # La transicion es suave, sin saltos
        err_abs = abs(self.error)
        kp = self.kp_low + (self.kp_high - self.kp_low) * err_abs
        # Control proporcional
        w = -kp * self.error
        w = np.clip(w, -self.max_w, self.max_w)
        v = self.v_base
        v = np.clip(v, -self.max_v, self.max_v)
        # Factor del semaforo (solo afecta v, no w)
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
