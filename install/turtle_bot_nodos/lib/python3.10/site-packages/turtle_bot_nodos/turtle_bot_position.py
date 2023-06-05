#! /usr/bin/env python

import serial,time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import math


class Position(Node):

    def __init__(self):

        super().__init__('turtle_bot_position')

        self.subscription_Ticks = self.create_subscription(String, 'turtlebot_route', self.listener_callback, 10)  
        #Creamos el publisher al tópico '/turtlebot_position':
        self.cmd_publisher = self.create_publisher(Twist,'turtlebot_position',10)

        self.subscriptionTwist = self.create_subscription(Twist,'/turtlebot_cmdVel',self.publicar_pos,10)


        self.x = 0
        self.y = 0
        self.phi = 0

    def listener_callback(self, msg):

        vel = msg.data.split(sep=";")
        self.duration = float(vel[0])
        self.linear = float(vel[1])
        self.angular = duration = float(vel[2])


        self.x,self.y = self.robotPosition(self.duration,self.linear,self.angular)

        twist_mss = Twist()
        twist_mss.linear.x = float(self.x)
        twist_mss.linear.y = float(self.y)
        self.cmd_publisher.publish(twist_mss)

    def publicar_pos(self,msg):
        twist_mss = Twist()
        twist_mss.linear.x = float(self.x)
        twist_mss.linear.y = float(self.y)
        self.cmd_publisher.publish(twist_mss)

    def robotPosition(self,duration,linear,angular):
        difx = 0
        dify = 0
        difphi = 0

        if self.linear != 0:
             difx = linear * math.cos(self.phi)*duration
             dify = linear * math.sin(self.phi)*duration

        x = self.x + difx
        self.x = x
        y = self.y + dify
        self.y = y

        if self.angular != 0:
            difphi = (2/17)*angular*duration

        phi = self.phi + difphi
        self.phi = phi

        return x,y

def main(args=None):
        rclpy.init(args=args)
        position_node = Position()

        rclpy.spin(position_node)
    
        Position.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

        main()

