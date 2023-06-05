#! /usr/bin/env python
import rclpy
from rclpy.node import Node

import tkinter as tk
from tkinter import filedialog
from geometry_msgs.msg import Twist
from time import perf_counter
from servicios.srv import ReproduceRoute
class player(Node):
        
    def __init__(self):
        super().__init__("turtle_bot_virtual")
        self.file_path=""
        self.get_logger().info("turtle_bot_player has been started correctly.")
        
    def get_route(self):
        root = tk.Tk()
        root.withdraw()
        self.file_path = filedialog.askopenfilename(
            defaultextension=".txt",
            filetypes=[("Archivo TXT","*.txt")])
        print(f"[INFO] Usted selecciono la ruta: {self.file_path}")    
        
    def callback_get_route(self):
        cliente = self.create_client(ReproduceRoute,"/RP")
        while not cliente.wait_for_service(1.0):
            self.get_logger().info("---------------")
        
        request = ReproduceRoute.Request()
        request.file_path = self.file_path

        self.future = cliente.call_async(request)
        rclpy.spin_until_future_complete(self,future=self.future)

        self.estado=self.future.result().ruta
        self.get_logger().info(self.estado) 
        self.get_logger().info('Funciona <3 <3 <3') 

def main(args=None):
    rclpy.init(args=args)
    
    player_node = player()
    
    player_node.get_route()
    player_node.callback_get_route()

    rclpy.spin(player_node)
    
    player.destroy_node()
    rclpy.shutdown()
    
if __name__== "__main__":
    main()