#! /usr/bin/env python
import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import filedialog

from time import perf_counter

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from servicios.srv import ReproduceRoute

class route_saver(Node):
    def __init__(self):
        """"
        Constructor de la clase 'route_saver'. Crea el nodo con nombre 'route_saver', el cual pública en el tópico
         '/turtlebot_cmdVel'. Tambien crea el servicio encargado de reproducir la ruta contenida en un archico .txt
        """
        super().__init__('route_saver')
        self.cmd_publisher = self.create_publisher(Twist,'/turtlebot_cmdVel',10)
        self.cmd_publisher_route = self.create_publisher(String,'/turtlebot_route',10)
        self.srv = self.create_service(ReproduceRoute, "/RP", self.replicate_route_callback)

    def replicate_route_callback(self, request, response):
        """"
        Respuesta del llamado del servicio.
        Dado un path por paŕametro en 'request', se réplica la ruta que describe el archivo txt del path indicado.
        
        Args:
            request: clase asociada a las entradas del servicio
            response:  clase asociada a las salidas del servicio    
        """
        self.file_path = request.file_path
        self.read_keys()
        
        response.ruta= "esta correcto :))"                                      
        print('[INFO] Se replico con éxito la ruta.') 
        return response
        
    def replicate_route(self,lineal,angular):
        """Cuando se presiona una tecla en el teclado, si es 'w','a','s' o 'd' asigna un valor a las variables
        a y l que multiplican por 1 o -1 las velocidades lineales y angulares respectivamente.
        
        Args:
            key: tecla presionada en el teclado
        """

        twist_mss = Twist()
        twist_mss.linear.x = lineal
        twist_mss.angular.z = angular
        self.cmd_publisher.publish(twist_mss)
            
    def stops_movement(self):
        """Detiene el movimiento del robot cuando se deja de presionar una tecla. Publica el mensaje tipo
        Twist en el tópico '/turtlebot_cmdVel' con velocidad lineal y angular en cero."""

        twist_mss = Twist()
        twist_mss.linear.x = 0.0
        twist_mss.angular.z = 0.0
        self.cmd_publisher.publish(twist_mss)

    def read_keys(self):
        """"
        Dado el path del archivo txt, abre el archivo y lee cada linea. Luego, utiliza la función 
        'replicate_route' con la información de cada linea.
            key: letra que contiene la linea actual
            duration: tiempo que se presiono la letra
        """
        f = open(self.file_path)
        lines = f.readlines()
        for l in lines:
            string_mss = String()
            string_mss.data = l.split(sep="\n")[0]
            self.cmd_publisher_route.publish(string_mss)

            ll = l.split(sep=';')
            duration = float(ll[0])
            linear_vel = float(ll[1])
            angular_vel = float(ll[2].split(sep="\n")[0])

            print(f"[INFO] Duración: {duration} / Velocidad: {linear_vel}, {angular_vel}")

            current_time = perf_counter()
            diff = 0
            while diff <= duration:
                self.replicate_route(linear_vel,angular_vel)
                diff = perf_counter()-current_time
            self.stops_movement()

def main(args=None):
    
    rclpy.init(args=args)
    route = route_saver()

    while rclpy.ok():
        rclpy.spin_once(route)

    route.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()