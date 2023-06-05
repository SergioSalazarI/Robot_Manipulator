#! /usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from pynput import keyboard as kb

import tkinter as tk
from tkinter import filedialog

from time import perf_counter
import serial,time

class teleop(Node):
    """Crea objetos de tipo nodo."""
    
    def __init__(self):
        """Constructor de la clase teleop."""

        super().__init__("robot_manipulator_teleop")

        #Creamos el publisher al tópico '/turtlebot_cmdVel':
        self.cmd_publisher = self.create_publisher(Twist,'/robot_manipulator_instructions',10)
        self.get_logger().info("Robot Manipulator Teleop has been started correctly.")

    def receive_parameters(self):
        """Pide los parámetros de velocidad lineal y angular al usuario y los publica en el tópico '/turtlebot_route'"""
        
        self.pos_x = float(input("[INFO] Indique la posición en X deseada:"))
        self.pos_y = float(input("[INFO] Indique la posición en Y deseada:"))
        self.pos_z = float(input("[INFO] Indique la posición en Z deseada:"))

        #self.manipulator_callback()

    def manipulator_callback(self):
        """Multiplica la velocidad lineal y angular por -1 o 1 dependiendo de la tecla presionada. Publica el
        mensaje tipo Twist en el tópico '/turtlebot_cmdVel'."""

        twist_mss = Twist()
        twist_mss.linear.x = self.pos_x 
        twist_mss.linear.y = self.pos_y 
        twist_mss.linear.z = self.pos_z 
        self.cmd_publisher.publish(twist_mss)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = teleop()
    
    teleop_node.receive_parameters()
    
    rclpy.spin_until_future_complete(teleop_node,teleop_node.manipulator_callback())
    
    teleop.destroy_node()
    rclpy.shutdown()
    
if __name__== "__main__":
    main()