#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()

        # Tamaño chico para que vaya rapido
        self.width = 160
        self.height = 120

        # ROI mas grande: ve mas adelante y mas ancho para anticipar curvas
        self.roi_top = 0.15
        self.roi_bot = 0.90
        self.roi_left = 0.10
        self.roi_right = 0.90

        # Threshold simple (mucho mas rapido que adaptive)
        # Valores menores a thresh_val se vuelven blancos (la linea negra)
        self.thresh_val = 80

        # Area minima (bajita para detectar lineas finas tipo plumon)
        self.min_area = 20

        # Suavizado del error
        self.error_filt = 0.0
        self.alpha = 0.25  # cuanto pesa el error nuevo (mas bajo = mas suavizado)

        self.cv_img = None

        # Camara del robot real
        self.create_subscription(Image, '/video_source/raw', self.image_cb, 10)

        # Publicamos error, deteccion e imagen
        self.error_pub = self.create_publisher(Float32, '/line_error', 10)
        self.detected_pub = self.create_publisher(Bool, '/line_detected', 10)
        self.debug_pub = self.create_publisher(Image, '/line_stage', 10)

        self.get_logger().info("Seguidor de linea listo")

    def image_cb(self, msg):
        # Procesamos directo en el callback
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.resize(frame, (self.width, self.height))

        # Recorte
        y1 = int(self.height * self.roi_top)
        y2 = int(self.height * self.roi_bot)
        x1 = int(self.width * self.roi_left)
        x2 = int(self.width * self.roi_right)
        roi = frame[y1:y2, x1:x2]

        # Grises + blur ligero
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Threshold simple (linea oscura queda blanca)
        _, mask = cv2.threshold(gray, self.thresh_val, 255, cv2.THRESH_BINARY_INV)

        # Solo dilatamos para reforzar la linea (sin erode para no borrar lineas delgadas)
        mask = cv2.dilate(mask, None, iterations=2)

        # Buscamos contornos
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        error_raw = 0.0

        if len(cnts) > 0:
            # Solo contornos grandes (ignoramos manchas)
            valid = [c for c in cnts if cv2.contourArea(c) > self.min_area]

            if len(valid) > 0:
                # El mas grande de los validos
                c = max(valid, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    roi_w = x2 - x1
                    error_raw = (cx - roi_w / 2) / (roi_w / 2)
                    detected = True
                    cv2.circle(frame, (cx + x1, (y1+y2)//2), 4, (0, 0, 255), -1)

        # Suavizado del error
        if detected:
            self.error_filt = (1 - self.alpha) * self.error_filt + self.alpha * error_raw

        # Dibujamos rectangulo de la ROI
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

        # Publicamos
        err_msg = Float32()
        err_msg.data = float(self.error_filt)
        self.error_pub.publish(err_msg)

        det_msg = Bool()
        det_msg.data = detected
        self.detected_pub.publish(det_msg)

        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.debug_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

