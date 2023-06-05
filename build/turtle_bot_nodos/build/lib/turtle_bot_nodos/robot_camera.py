#! /usr/bin/env python
import rclpy
from rclpy.node import Node

import cv2

from geometry_msgs.msg import Twist
from std_msgs.msg import String

#from servicios.srv import Start_perception_test

class camera(Node):
    def __init__(self):
        """"
        Constructor de la clase 'route_saver'. Crea el nodo con nombre 'route_saver', el cual pública en el tópico
         '/turtlebot_cmdVel'. Tambien crea el servicio encargado de reproducir la ruta contenida en un archico .txt
        """
        super().__init__('route_saver')
        self.cmd_publisher = self.create_publisher(Twist,'/turtlebot_cmdVel',10)
        self.cmd_publisher_route = self.create_publisher(String,'/turtlebot_route',10)
        #self.srv = self.create_service(Start_perception_test, "/Start_perception_test", self.start_perception_test)

    def turn_on(self):
        img = cv2.imread("/home/sergio/Pictures/Webcam/tester_triangulo.jpg")

        # converting image into grayscale image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print(cnts)
        while True:
            cv2.imshow("Sheep", thresh)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()  # to exit from all the processes

    def detect_shapes(self,img):
        a=0
        return a

    def replicate_route_callback(self, request, response):
        """"
        Respuesta del llamado del servicio.
        Dado un path por paŕametro en 'request', se réplica la ruta que describe el archivo txt del path indicado.
        
        Args:
            request: clase asociada a las entradas del servicio
            response:  clase asociada a las salidas del servicio    
        """
        self.banner_a = request.banner_a
        self.banner_b = request.banner_b
        self.read_keys()
        
        response.ruta= "esta correcto :))"                                      
        print('[INFO] Se replico con éxito la ruta.') 
        return response

def main(args=None):
    
    rclpy.init(args=args)
    camera_node = camera()
    camera_node.turn_on()

    while rclpy.ok():
        rclpy.spin_once(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()