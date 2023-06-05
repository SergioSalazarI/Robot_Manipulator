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

        super().__init__("turtle_bot_teleop")
        self.current_key = 'q'

        #Creamos el publisher al tópico '/turtlebot_cmdVel':
        self.cmd_publisher = self.create_publisher(Twist,'/turtlebot_cmdVel',10)
        self.get_logger().info("Turtle Teleop has been started correctly.")

        #Creamos el publisher al tópico '/turtlebot_route':
        self.cmd_publisher_route = self.create_publisher(String,'/turtlebot_route',10)

    def print_instructions(self):
        """Imprime en la terminal las instrucciones de uso."""

        print("________________________________________________________________")
        print("                        Instrucciones")
        print("    Presione 'W' para ir hacia adelante.")
        print("    Presione 'S' para ir hacia atras.")
        print(f"    Presione 'D' para rotar {self.angular_vel} grados a la derecha.")
        print(f"    Presione 'A' para rotar {self.angular_vel} grados a la izquierda.")
        print("________________________________________________________________")

        print("    Para mover la juntura deseada presione las teclas según se indica.")
        print("--------------------------JUNTURA 1-------------------------")
        print("         Presione 'T' para ir hacia adelante.")
        print("         Presione 'F' para ir hacia atras.")
        print("--------------------------JUNTURA 2-------------------------")
        print("         Presione 'Y' para ir hacia adelante.")
        print("         Presione 'G' para ir hacia atras.")
        print("--------------------------JUNTURA 3-------------------------")
        print("         Presione 'U' para ir hacia adelante.")
        print("         Presione 'H' para ir hacia atras.")
        print("--------------------------JUNTURA 4-------------------------")
        print("         Presione 'I' para ir hacia adelante.")
        print("         Presione 'J' para ir hacia atras.")
        print("________________________________________________________________")
        
    def receive_parameters(self):
        """Pide los parámetros de velocidad lineal y angular al usuario y los publica en el tópico '/turtlebot_route'"""
        
        self.linear_vel = float(input("[INFO] Indique la velocidad lineal deseada:"))
        self.angular_vel = float(input("[INFO] Indique la velocidad angular deseada:"))

        #s = String()
        #s.data = f"{self.linear_vel}\n{self.angular_vel}"

        #self.cmd_publisher_route.publish(s)
        self.print_instructions()
        
    def key_callback(self,a,l):
        """Multiplica la velocidad lineal y angular por -1 o 1 dependiendo de la tecla presionada. Publica el
        mensaje tipo Twist en el tópico '/turtlebot_cmdVel'."""

        twist_mss = Twist()
        twist_mss.linear.x = a*self.linear_vel #a=1 adelante
        twist_mss.angular.z = l*self.angular_vel #l=1 derecha
        self.cmd_publisher.publish(twist_mss)

    def key_callback_brazo(self,vel):
        twist_mss = Twist()
        twist_mss.linear.y = float(vel)
        self.cmd_publisher.publish(twist_mss)
        
    def stops_movement(self):
        """Detiene el movimiento del robot cuando se deja de presionar una tecla. Publica el mensaje tipo
        Twist en el tópico '/turtlebot_cmdVel' con velocidad lineal y angular en cero."""

        twist_mss = Twist()
        twist_mss.linear.x = 0.0
        twist_mss.angular.z = 0.0
        self.cmd_publisher.publish(twist_mss)

    def on_press(self,key):
        """Cuando se presiona una tecla en el teclado, si es 'w','a','s' o 'd' asigna un valor a las variables
        a y l que multiplican por 1 o -1 las velocidades lineales y angulares respectivamente.
        
        Args:
            key: tecla presionada en el teclado
        """
        try:
            pos_orientation = ['t','y','u','i']
            neg_orientation = ['f','g','h','j']

            vel = 0

            if key.char in pos_orientation:
                print(f'POSITIVO {key.char}')
                if key.char == 't':
                    vel = 116
                elif key.char =='y':
                    vel = 121
                elif key.char == 'u':
                    vel = 117
                else:
                    vel = 105
                self.key_callback_brazo(vel)

            elif key.char in neg_orientation:
                print("negativo")
                if key.char == 'f':
                    vel = 102
                elif key.char =='g':
                    vel = 103
                elif key.char == 'h':
                    vel = 104
                else:
                    vel = 106
                self.key_callback_brazo(vel)

            elif key.char in ['w','a','s','d']:
                a = 0
                l = 0
                if key.char == 'w':
                    a = 1
                elif key.char =='s':
                    a = -1
                elif key.char == 'd':
                    l = -1
                else:
                    l = 1
                self.key_callback(a,l)
            else:
                print("[INFO] La tecla presionada no tiene un movimiento asociado \n Siga las instrucciones.")
                
            if key.char != self.current_key:
                self.current_time = perf_counter()
                self.current_key = key.char
        except:
            print("Caracter especial no identificado.")
        
    def on_release(self,key):
        """Cuando se deja de presionar la tecla, llama a la función que detiene el movimiento del robot.
        Publica en el tópico '/turtlebot_route' el String que contiene la tecla presionada y el tiempo que
        se presionó.
        
        Args:
            key: tecla que se dejo de presionar en el teclado.
        """
        if key.char in ['w','a','s','d']:
            a = 0
            l = 0
            if key.char == 'w':
                a = 1
            elif key.char =='s':
                a = -1
            elif key.char == 'd':
                l = -1
            else:
                l = 1

            diff = perf_counter()-self.current_time
            self.current_time = perf_counter()

            #Crea el mensaje tipo String y lo publica en el tópico:
            string_mss = String()
            string_mss.data = f"{diff};{a*self.linear_vel};{l*self.angular_vel}"
            string_mss.data = f"{key.char};{diff};{a*self.linear_vel};{l*self.angular_vel}"
            self.cmd_publisher_route.publish(string_mss)
        
        self.stops_movement()
        
    def listen_keyboard(self):
        """Ecucha el teclado y esta a la espera de que se presione una tecla para ejecutar alguna función."""
        with kb.Listener(on_press=self.on_press,on_release=self.on_release) as listener:
            listener.join()

def main(args=None):
    rclpy.init(args=args)
    teleop_node = teleop()
    
    teleop_node.receive_parameters()
    teleop_node.listen_keyboard()
    
    rclpy.spin(teleop_node)
    
    teleop.destroy_node()
    rclpy.shutdown()
    
if __name__== "__main__":
    main()