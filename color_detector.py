#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Puente entre ROS y OpenCV
        self.bridge = CvBridge()

        # Tamaño al que reducimos la imagen para que vaya mas rapido
        self.width = 320
        self.height = 240

        # Radio minimo para no agarrar ruido
        self.min_radius = 10

        # Radio para considerar que la esfera esta cerca (a 1m o menos)
        # Lo saque viendo a que radio se veian las esferas a 1m en el simlador
        self.close_radius = 40

        # Rangos HSV para cada color
        self.red_low = np.array([0, 150, 100])
        self.red_high = np.array([10, 255, 255])

        self.green_low = np.array([45, 150, 100])
        self.green_high = np.array([85, 255, 255])

        self.yellow_low = np.array([20, 150, 150])
        self.yellow_high = np.array([35, 255, 255])

        # Recibimos la imagen del line_follower (ya viene con el punto de la linea)
        self.create_subscription(Image, '/line_stage', self.image_cb, 10)

        # Publicamos la imagen procesada y color que detectamos
        self.img_pub = self.create_publisher(Image, '/processed_img', 10)
        self.color_pub = self.create_publisher(String, '/detected_color', 10)

        self.get_logger().info("Detector de color listo")

    def image_cb(self, msg):
        # Pasamos de ROS a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # A HSV porque ahi es mas facil filtrar por color
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Mascara de cada color
        mask_red = self.get_mask(hsv, self.red_low, self.red_high)
        mask_green = self.get_mask(hsv, self.green_low, self.green_high)
        mask_yellow = self.get_mask(hsv, self.yellow_low, self.yellow_high)

        # Sacamos el radio de cada uno (0 si esta lejos o no se ve)
        r_red = self.detect_and_draw(frame, mask_red)
        r_green = self.detect_and_draw(frame, mask_green)
        r_yellow = self.detect_and_draw(frame, mask_yellow)

        # Decidimos cual color manda
        dominant = self.get_dominant_color(r_red, r_yellow, r_green)

        # Publicamos el color
        color_msg = String()
        color_msg.data = dominant
        self.color_pub.publish(color_msg)

        # Publicamos la imagen con los circulos dibujados
        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.img_pub.publish(out_msg)

    def get_dominant_color(self, r_red, r_yellow, r_green):
        # Si no vemos nada cerca, no hay color
        if r_red == 0 and r_yellow == 0 and r_green == 0:
            return "none"

        # El rojo siempre gana, es el mas importante
        if r_red > 0:
            return "red"

        # Si no hay rojo, comparamos amarillo vs verde
        if r_yellow >= r_green:
            return "yellow"
        return "green"

    def get_mask(self, hsv, low, high):
        # Filtro de color + limpiamos ruido
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.GaussianBlur(mask, (9, 9), 0)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return mask

    def detect_and_draw(self, frame, mask):
        # Buscamos los contornos del color
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(cnts) == 0:
            return 0

        # Nos quedamos con el contorno mas grande
        c = max(cnts, key=cv2.contourArea)

        # Circulo minimo que lo encierra
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        # Si es muy chico es ruido
        if radius < self.min_radius:
            return 0

        # Si esta lejos lo ignoramos, no nos sirve
        if radius < self.close_radius:
            return 0

        # Esta cerca, lo dibujamos
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

        return radius


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
