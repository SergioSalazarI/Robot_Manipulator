import cv2
import numpy as np
import serial

import pygame, sys,random, time , math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class interface(Node):
    """Crea objetos de tipo nodo."""
    
    def __init__(self):
        """Constructor de la clase interface."""

        super().__init__("robot_manipulator_interface")
        self.subscription = self.create_subscription(Twist, '/manipulator_angles', self.listener_callback, 10)

        self.pi = math.pi
        self.i=-300
        self.gama=self.pi/6
        self.inc=0
        self.inc2=0
        self.teta=0
        self.beta=0

        #Colores
        self.BLACK2=(56,56,56)
        self.BLACK =(0,0,0)
        self.WHITE=(255,255,255)
        self.GREEN=(0,255,0)
        self.RED=(255,0,0)
        self.RED2=(100, 150, 150)
        self.BLUE=(0,0,255)
        self.YELLOW=(255,255,0)
        self.YELLOW2=(180,180,0)

        self.Q1 = 0
        self.Q2 = 0
        self.Q3 = 0

        self.gripper_positions = []

    def listener_callback(self, msg):
        self.Q1 = msg.linear.x
        self.Q2 = msg.linear.y
        self.Q3 = msg.linear.z
        print(f"Q1={self.Q1} Q2={self.Q2} Q3={self.Q3}")
    
    def interface(self):
        pygame.init()  #iniciamos pygame
        pygame.mixer.init()  #Herramienta de audio

        prev_Q1 = self.Q1
        prev_Q2 = self.Q2
        prev_Q3 = self.Q3

        #x,y
        size =(700,700) #definimos Tamaño de Ventana
        clock= pygame.time.Clock() #Creamos el Tiempo del videojuego
        screen=pygame.display.set_mode(size)
        pygame.display.set_caption("Manipulador")

        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            for event in pygame.event.get():    #AQUI TIENE LUGAR EL Motor Gráfico
                if event.type==pygame.QUIT:     #CONDICION QUE PERMITE CERRAR EL PROGRAMA
                    sys.exit()
                if event.type==pygame.KEYDOWN:
                    if event.key==pygame.K_LEFT:
                        self.inc=+0.01
                    if event.key==pygame.K_RIGHT:
                        self.inc=-0.01
                    if event.key == pygame.K_UP:
                        self.inc2=+0.01
                    if event.key==pygame.K_DOWN:
                        self.inc2=-0.01
                    #AL DEJAR DE PRECIONAR
                if event.type==pygame.KEYUP:
                    if event.key==pygame.K_LEFT:
                        self.inc=0
                    if event.key==pygame.K_RIGHT:
                        self.inc=0
                    if event.key==pygame.K_UP:
                        self.inc2=0
                    if event.key==pygame.K_DOWN:
                        self.inc2=0

            self.teta+=self.inc; #rotaciones sobre el eje Z
            self.beta+=self.inc2; #rotacion sobre el eje X
            screen.fill(self.BLACK2) #Fondo del dibujo

            #dibujamos los ejes de referencia
            x_nega=self.pixel(-300,0,0,self.gama,self.teta,self.beta,1)
            x_posi=self.pixel(300,0,0,self.gama,self.teta,self.beta,1)
            y_nega=self.pixel(0,-300,0,self.gama,self.teta,self.beta,1)
            y_posi=self.pixel(0,300,0,self.gama,self.teta,self.beta,1)
            z_nega=self.pixel(0,0,-300,self.gama,self.teta,self.beta,1)
            z_posi=self.pixel(0,0,300,self.gama,self.teta,self.beta,1)    
                                        #x,y1    x,y2
            pygame.draw.line(screen,self.GREEN,x_nega,x_posi) # eje horizontal X
            pygame.draw.line(screen,self.BLUE,y_nega,y_posi)  # eje vertical   Y  
            pygame.draw.line(screen,self.RED,z_nega,z_posi)  # eje vertical   Z  
            

            if self.Q1 != prev_Q1 or self.Q2 != prev_Q2 or self.Q3 != prev_Q3:
                num_frames = 30
                for frame in range(num_frames):
                    t = frame / (num_frames - 1)
                    Q1 = prev_Q1 * (1 - t) + self.Q1 * t
                    Q2 = prev_Q2 * (1 - t) + self.Q2 * t
                    Q3 = prev_Q3 * (1 - t) + self.Q3 * t

                    screen.fill(self.BLACK2)

                    (x,y,z)=self.gripper_position(Q1,Q2,Q3)
                    (x,y) = self.pixel(x,y,z,self.gama,self.teta,self.beta,1)
                    self.gripper_positions.append((x, y))

                    self.paint(Q1,Q2,Q3,screen)

                    pygame.display.flip()
                    clock.tick(48)

                prev_Q1 = self.Q1
                prev_Q2 = self.Q2
                prev_Q3 = self.Q3

            else:

                Q1 = prev_Q1 
                Q2 = prev_Q2 
                Q3 = prev_Q3
                (x,y,z)=self.gripper_position(Q1,Q2,Q3)
                (x,y) = self.pixel(x,y,z,self.gama,self.teta,self.beta,1)
                self.gripper_positions.append((x, y))
                self.paint(Q1,Q2,Q3,screen)

                pygame.display.flip()
                clock.tick(48)
            
    def paint(self,Q1,Q2,Q3,screen):

        Q4 = 0  
        #las convertimos a grados
        q1=(Q1-90)*self.pi/180
        q2=(Q2-45)*self.pi/180
        q3=(Q3)*self.pi/180
        q4=(Q4-90)*self.pi/180

        escala=8
        
        #Dibujamos al robot
        self.eslabon_1 (screen,self.gama,self.teta,self.beta,escala)
        self.eslabon_2 (screen,self.gama,self.teta,self.beta,escala,q1+self.pi/2)
        self.eslabon_q2 (screen,self.gama,self.teta,self.beta,escala,q1,q2)
        self.eslabon_q3 (screen,self.gama,self.teta,self.beta,escala,q1,q2,q3)
        self.robot_esqueleto(screen,self.gama,self.teta,self.beta,escala,q1,q2+self.pi/2,q3,q4)

        if len(self.gripper_positions) > 1:
            for i in range(len(self.gripper_positions) - 1):
                x1, y1 = self.gripper_positions[i]
                x2, y2 = self.gripper_positions[i+1]
                pygame.draw.line(screen, self.RED, (x1+100, y1+100), (x2+100, y2+100), 4)
                print(f"estoy pintandooo de ({x1},{y1}) a ({x2},{y2})")

    def gripper_position(self, Q1, Q2, Q3):
        # Arm dimensions
        L1=10.48
        L2=16.43
        L3=9.27

        # Convert joint angles from degrees to radians
        q1 = math.radians(Q1)
        q2 = math.radians(Q2)
        q3 = math.radians(Q3)

        q2=q2+math.pi/2
        x=(L2*math.cos(q2) +L3*math.cos(q2+q3)) *math.cos(q1)
        y=(L2*math.cos(q2) +L3*math.cos(q2+q3)) *math.sin(q1)
        z=L1+L2*math.sin(q2) +L3*math.sin(q2+q3)

        return (x, y, z)
    
    def pixel(self,x,y,z,gama,teta,beta,escala): #funcion de ubicación en la pantalla
        x=x*escala
        y=y*escala
        z=z*escala

        #transformación RZ
        x2=(x*math.cos(teta) - y*math.sin(teta))
        y2=(y*math.cos(teta) + x*math.sin(teta))
        z2=z

        #transformacion en Rx
        x3= x2
        y3= y2*math.cos(beta) - z2*math.sin(beta)
        z3= z2*math.cos(beta) + y2*math.sin(beta)
        
        
        #print(x,y,z,"Reales")
        x_screen=int(  -x3*math.cos(gama) + y3*math.cos(gama))
        y_screen=int( -x3*math.sin(gama) -y3*math.sin(gama) +z3)
        
        x_screen=(x_screen+400)
        y_screen=(abs(y_screen-400))

        return (x_screen,y_screen)
    
    def cd_eslabon2(self,xf,yf,zf,L1,L2,q1,q2):
        x=L2*math.cos(q1)*math.cos(q2 + self.pi/2) - yf*math.sin(q1) + xf*math.cos(q1)*math.cos(q2 + self.pi/2) - zf*math.cos(q1)*math.sin(q2 + self.pi/2)
        y=yf*math.cos(q1) + L2*math.cos(q2 + self.pi/2)*math.sin(q1) + xf*math.cos(q2 + self.pi/2)*math.sin(q1) - zf*math.sin(q1)*math.sin(q2 + self.pi/2) 
        z=L1 + L2*math.sin(q2 + self.pi/2) + zf*math.cos(q2 + self.pi/2) + xf*math.sin(q2 + self.pi/2)
        return (x,y,z);

    def cd_eslabon3(self,tx,ty,tz,L1,L2,L3,q1,q2,q3):
        x=L2*math.cos(q1)*math.cos(q2 + self.pi/2) - tx*(math.cos(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q1)*math.cos(q3)*math.cos(q2 + self.pi/2)) - tz*(math.cos(q1)*math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q1)*math.cos(q2 + self.pi/2)*math.sin(q3)) - ty*math.sin(q1) - L3*(math.cos(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q1)*math.cos(q3)*math.cos(q2 + self.pi/2))
        y=ty*math.cos(q1) - tx*(math.sin(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q3)*math.cos(q2 + self.pi/2)*math.sin(q1)) - tz*(math.cos(q3)*math.sin(q1)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q1)*math.sin(q3)) - L3*(math.sin(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q3)*math.cos(q2 + self.pi/2)*math.sin(q1)) + L2*math.cos(q2 + self.pi/2)*math.sin(q1)
        z=L1 + L3*(math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q3)) + tx*(math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q3)) + tz*(math.cos(q3)*math.cos(q2 + self.pi/2) - math.sin(q3)*math.sin(q2 + self.pi/2)) + L2*math.sin(q2 + self.pi/2)
        return (x,y,z)

    def cd_eslabon4(self,tx,ty,tz,L1,L2,L3,L4,q1,q2,q3,q4):
        x=L2*math.cos(q1)*math.cos(q2 + self.pi/2) - L4*(math.cos(q4)*(math.cos(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q1)*math.cos(q3)*math.cos(q2 + self.pi/2)) + math.sin(q4)*(math.cos(q1)*math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q1)*math.cos(q2 + self.pi/2)*math.sin(q3))) - tx*(math.cos(q4)*(math.cos(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q1)*math.cos(q3)*math.cos(q2 + self.pi/2)) + math.sin(q4)*(math.cos(q1)*math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q1)*math.cos(q2 + self.pi/2)*math.sin(q3))) - tz*(math.cos(q4)*(math.cos(q1)*math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q1)*math.cos(q2 + self.pi/2)*math.sin(q3)) - math.sin(q4)*(math.cos(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q1)*math.cos(q3)*math.cos(q2 + self.pi/2))) - ty*math.sin(q1) - L3*(math.cos(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q1)*math.cos(q3)*math.cos(q2 + self.pi/2))
        y=ty*math.cos(q1) - L4*(math.cos(q4)*(math.sin(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q3)*math.cos(q2 + self.pi/2)*math.sin(q1)) + math.sin(q4)*(math.cos(q3)*math.sin(q1)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q1)*math.sin(q3))) - tx*(math.cos(q4)*(math.sin(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q3)*math.cos(q2 + self.pi/2)*math.sin(q1)) + math.sin(q4)*(math.cos(q3)*math.sin(q1)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q1)*math.sin(q3))) - tz*(math.cos(q4)*(math.cos(q3)*math.sin(q1)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q1)*math.sin(q3)) - math.sin(q4)*(math.sin(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q3)*math.cos(q2 + self.pi/2)*math.sin(q1))) - L3*(math.sin(q1)*math.sin(q3)*math.sin(q2 + self.pi/2) - math.cos(q3)*math.cos(q2 + self.pi/2)*math.sin(q1)) + L2*math.cos(q2 + self.pi/2)*math.sin(q1)
        z=L1 + tx*(math.cos(q4)*(math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q3)) + math.sin(q4)*(math.cos(q3)*math.cos(q2 + self.pi/2) - math.sin(q3)*math.sin(q2 + self.pi/2))) + tz*(math.cos(q4)*(math.cos(q3)*math.cos(q2 + self.pi/2) - math.sin(q3)*math.sin(q2 + self.pi/2)) - math.sin(q4)*(math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q3))) + L3*(math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q3)) + L2*math.sin(q2 + self.pi/2) + L4*(math.cos(q4)*(math.cos(q3)*math.sin(q2 + self.pi/2) + math.cos(q2 + self.pi/2)*math.sin(q3)) + math.sin(q4)*(math.cos(q3)*math.cos(q2 + self.pi/2) - math.sin(q3)*math.sin(q2 + self.pi/2)))
        return (x,y,z)

    
    def cd_2 (self,q1,q2,l1,l2):
        x=l2*math.cos(q2)*math.cos(q1)
        y=l2*math.cos(q2)*math.sin(q1)
        z=l1+l2*math.sin(q2)
        return (x,y,z)

    def cd_3 (self,q1,q2,q3,l1,l2,l3):
        x=(l2*math.cos(q2) +l3*math.cos(q2+q3)) *math.cos(q1)
        y=(l2*math.cos(q2) +l3*math.cos(q2+q3)) *math.sin(q1)
        z=l1+l2*math.sin(q2) +l3*math.sin(q2+q3)
        return (x,y,z)

    #librería Shoubi V3
    def robot_esqueleto(self,screen,gama,teta,beta,escala,q1,q2,q3,q4):
        #Longitud de los eslabones
        l1=10.48
        l2=16.43
        l3=9.27

        #Puntos de control
        A=(0,0,0)
        B=(0,0,l1)
        C=self.cd_2(q1,q2,l1,l2)
        D=self.cd_3(q1,q2,q3,l1,l2,l3)

        #Convertimos R3 a R2 Virtual
        A2=self.pixel(A[0],A[1],A[2],gama,teta,beta,escala)
        B2=self.pixel(B[0],B[1],B[2],gama,teta,beta,escala)
        C2=self.pixel(C[0],C[1],C[2],gama,teta,beta,escala)
        D2=self.pixel(D[0],D[1],D[2],gama,teta,beta,escala)

        #los dibujamos y luego los unimos
        pygame.draw.circle(screen, self.RED2, (A2) , 3)
        pygame.draw.circle(screen, self.RED2, (B2) , 3)
        pygame.draw.circle(screen, self.RED2, (C2) , 3)
        pygame.draw.circle(screen, self.RED2, (D2) , 3)
        
        pygame.draw.line(screen,self.RED2,A2,B2)
        pygame.draw.line(screen,self.RED2,B2,C2)
        pygame.draw.line(screen,self.RED2,C2,D2)

    def eslabon_1 (self,screen,gama,teta,beta,escala):
        A=self.pixel(5.3,0,0,gama,teta,beta,escala)
        B=self.pixel(5.3*math.cos(self.pi/8),5.3*math.sin(self.pi/8),0,gama,teta,beta,escala)
        C=self.pixel(5.3*math.cos(2*self.pi/8),5.3*math.sin(2*self.pi/8),0,gama,teta,beta,escala)
        D=self.pixel(5.3*math.cos(3*self.pi/8),5.3*math.sin(3*self.pi/8),0,gama,teta,beta,escala)
        E=self.pixel(5.3*math.cos(4*self.pi/8),5.3*math.sin(4*self.pi/8),0,gama,teta,beta,escala)
        F=self.pixel(5.3*math.cos(5*self.pi/8),5.3*math.sin(5*self.pi/8),0,gama,teta,beta,escala)
        G=self.pixel(5.3*math.cos(6*self.pi/8),5.3*math.sin(6*self.pi/8),0,gama,teta,beta,escala)
        H=self.pixel(5.3*math.cos(7*self.pi/8),5.3*math.sin(7*self.pi/8),0,gama,teta,beta,escala)
        I=self.pixel(5.3*math.cos(8*self.pi/8),5.3*math.sin(8*self.pi/8),0,gama,teta,beta,escala)
        J=self.pixel(5.3*math.cos(9*self.pi/8),5.3*math.sin(9*self.pi/8),0,gama,teta,beta,escala)
        K=self.pixel(5.3*math.cos(10*self.pi/8),5.3*math.sin(10*self.pi/8),0,gama,teta,beta,escala)
        L=self.pixel(5.3*math.cos(11*self.pi/8),5.3*math.sin(11*self.pi/8),0,gama,teta,beta,escala)
        M=self.pixel(5.3*math.cos(12*self.pi/8),5.3*math.sin(12*self.pi/8),0,gama,teta,beta,escala)
        N=self.pixel(5.3*math.cos(13*self.pi/8),5.3*math.sin(13*self.pi/8),0,gama,teta,beta,escala)
        O=self.pixel(5.3*math.cos(14*self.pi/8),5.3*math.sin(14*self.pi/8),0,gama,teta,beta,escala)
        P=self.pixel(5.3*math.cos(15*self.pi/8),5.3*math.sin(15*self.pi/8),0,gama,teta,beta,escala)
        AZ=self.pixel(5.3,0,5.7,gama,teta,beta,escala)
        BZ=self.pixel(5.3*math.cos(self.pi/8),5.3*math.sin(self.pi/8),5.7,gama,teta,beta,escala)
        CZ=self.pixel(5.3*math.cos(2*self.pi/8),5.3*math.sin(2*self.pi/8),5.7,gama,teta,beta,escala)
        DZ=self.pixel(5.3*math.cos(3*self.pi/8),5.3*math.sin(3*self.pi/8),5.7,gama,teta,beta,escala)
        EZ=self.pixel(5.3*math.cos(4*self.pi/8),5.3*math.sin(4*self.pi/8),5.7,gama,teta,beta,escala)
        FZ=self.pixel(5.3*math.cos(5*self.pi/8),5.3*math.sin(5*self.pi/8),5.7,gama,teta,beta,escala)
        GZ=self.pixel(5.3*math.cos(6*self.pi/8),5.3*math.sin(6*self.pi/8),5.7,gama,teta,beta,escala)
        HZ=self.pixel(5.3*math.cos(7*self.pi/8),5.3*math.sin(7*self.pi/8),5.7,gama,teta,beta,escala)
        IZ=self.pixel(5.3*math.cos(8*self.pi/8),5.3*math.sin(8*self.pi/8),5.7,gama,teta,beta,escala)
        JZ=self.pixel(5.3*math.cos(9*self.pi/8),5.3*math.sin(9*self.pi/8),5.7,gama,teta,beta,escala)
        KZ=self.pixel(5.3*math.cos(10*self.pi/8),5.3*math.sin(10*self.pi/8),5.7,gama,teta,beta,escala)
        LZ=self.pixel(5.3*math.cos(11*self.pi/8),5.3*math.sin(11*self.pi/8),5.7,gama,teta,beta,escala)
        MZ=self.pixel(5.3*math.cos(12*self.pi/8),5.3*math.sin(12*self.pi/8),5.7,gama,teta,beta,escala)
        NZ=self.pixel(5.3*math.cos(13*self.pi/8),5.3*math.sin(13*self.pi/8),5.7,gama,teta,beta,escala)
        OZ=self.pixel(5.3*math.cos(14*self.pi/8),5.3*math.sin(14*self.pi/8),5.7,gama,teta,beta,escala)
        PZ=self.pixel(5.3*math.cos(15*self.pi/8),5.3*math.sin(15*self.pi/8),5.7,gama,teta,beta,escala)
        
        #CARAS LATERALES
        pygame.draw.polygon(screen,self.YELLOW,(A,B,BZ,AZ))
        pygame.draw.polygon(screen,self.YELLOW,(B,C,CZ,BZ))
        pygame.draw.polygon(screen,self.YELLOW2,(C,D,DZ,CZ))
        pygame.draw.polygon(screen,self.YELLOW2,(D,E,EZ,DZ))

        pygame.draw.polygon(screen,self.YELLOW,(E,F,FZ,EZ))
        pygame.draw.polygon(screen,self.YELLOW,(F,G,GZ,FZ))
        pygame.draw.polygon(screen,self.YELLOW2,(G,H,HZ,GZ))
        pygame.draw.polygon(screen,self.YELLOW2,(H,I,IZ,HZ))

        pygame.draw.polygon(screen,self.YELLOW,(I,J,JZ,IZ))
        pygame.draw.polygon(screen,self.YELLOW,(J,K,KZ,JZ))
        pygame.draw.polygon(screen,self.YELLOW2,(K,L,LZ,KZ))
        pygame.draw.polygon(screen,self.YELLOW2,(L,M,MZ,LZ))

        pygame.draw.polygon(screen,self.YELLOW,(M,N,NZ,MZ))
        pygame.draw.polygon(screen,self.YELLOW2,(N,O,OZ,NZ))
        pygame.draw.polygon(screen,self.YELLOW,(O,P,PZ,OZ))

        #CARAS SUPERIORES
        pygame.draw.polygon(screen,self.YELLOW,(A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P),0)
        pygame.draw.polygon(screen,self.YELLOW,(AZ,BZ,CZ,DZ,EZ,FZ,GZ,HZ,IZ,JZ,KZ,LZ,MZ,NZ,OZ,PZ),0)
        pygame.draw.polygon(screen,self.BLACK,(AZ,BZ,CZ,DZ,EZ,FZ,GZ,HZ,IZ,JZ,KZ,LZ,MZ,NZ,OZ,PZ),1)
        
        return 0;

    def eslabon_2 (self,screen,gama,teta,beta,escala,q1):
        #definimos puntos en el espacio
        A= (-1.3,-1.3,5.7)
        B= (1.3 ,-1.3,5.7)
        C= (1.3 ,1.3 ,5.7)
        D= (-1.3,1.3 ,5.7)
    
        AZ= (-1.3,-1.3,11.9)
        BZ= (1.3 ,-1.3,11.9)
        CZ= (1.3 ,1.3 ,11.9)
        DZ= (-1.3,1.3 ,11.9)
        
        E=(-3*1.3,-1.3,9)
        F=(3*1.3 ,-1.3,9)
        G=(3*1.3 ,1.3 ,9)
        H=(-3*1.3,1.3 ,9)
        
        EZ=(-3*1.3,-1.3,11.9)
        FZ=(3*1.3 ,-1.3,11.9)
        GZ=(3*1.3 ,1.3 ,11.9)
        HZ=(-3*1.3,1.3 ,11.9)
        
        #Los Rotamos en RZ(q1)
        A=(A[0]*math.cos(q1) - A[1]*math.sin(q1),A[1]*math.cos(q1) + A[0]*math.sin(q1),A[2])
        B=(B[0]*math.cos(q1) - B[1]*math.sin(q1),B[1]*math.cos(q1) + B[0]*math.sin(q1),B[2])
        C=(C[0]*math.cos(q1) - C[1]*math.sin(q1),C[1]*math.cos(q1) + C[0]*math.sin(q1),C[2])
        D=(D[0]*math.cos(q1) - D[1]*math.sin(q1),D[1]*math.cos(q1) + D[0]*math.sin(q1),D[2])
        E=(E[0]*math.cos(q1) - E[1]*math.sin(q1),E[1]*math.cos(q1) + E[0]*math.sin(q1),E[2])
        F=(F[0]*math.cos(q1) - F[1]*math.sin(q1),F[1]*math.cos(q1) + F[0]*math.sin(q1),F[2])
        G=(G[0]*math.cos(q1) - G[1]*math.sin(q1),G[1]*math.cos(q1) + G[0]*math.sin(q1),G[2])
        H=(H[0]*math.cos(q1) - H[1]*math.sin(q1),H[1]*math.cos(q1) + H[0]*math.sin(q1),H[2])
        
        AZ=(AZ[0]*math.cos(q1) - AZ[1]*math.sin(q1),AZ[1]*math.cos(q1) + AZ[0]*math.sin(q1),AZ[2])
        BZ=(BZ[0]*math.cos(q1) - BZ[1]*math.sin(q1),BZ[1]*math.cos(q1) + BZ[0]*math.sin(q1),BZ[2])
        CZ=(CZ[0]*math.cos(q1) - CZ[1]*math.sin(q1),CZ[1]*math.cos(q1) + CZ[0]*math.sin(q1),CZ[2])
        DZ=(DZ[0]*math.cos(q1) - DZ[1]*math.sin(q1),DZ[1]*math.cos(q1) + DZ[0]*math.sin(q1),DZ[2])
        EZ=(EZ[0]*math.cos(q1) - EZ[1]*math.sin(q1),EZ[1]*math.cos(q1) + EZ[0]*math.sin(q1),EZ[2])
        FZ=(FZ[0]*math.cos(q1) - FZ[1]*math.sin(q1),FZ[1]*math.cos(q1) + FZ[0]*math.sin(q1),FZ[2])
        GZ=(GZ[0]*math.cos(q1) - GZ[1]*math.sin(q1),GZ[1]*math.cos(q1) + GZ[0]*math.sin(q1),GZ[2])
        HZ=(HZ[0]*math.cos(q1) - HZ[1]*math.sin(q1),HZ[1]*math.cos(q1) + HZ[0]*math.sin(q1),HZ[2])
        
        
        #Los convertimos a representación en R2
        A2= self.pixel(A[0],A[1],A[2],gama,teta,beta,escala)
        B2= self.pixel(B[0],B[1],B[2],gama,teta,beta,escala)
        C2= self.pixel(C[0],C[1],C[2],gama,teta,beta,escala)
        D2= self.pixel(D[0],D[1],D[2],gama,teta,beta,escala)
        E2= self.pixel(E[0],E[1],E[2],gama,teta,beta,escala)
        F2= self.pixel(F[0],F[1],F[2],gama,teta,beta,escala)
        G2= self.pixel(G[0],G[1],G[2],gama,teta,beta,escala)
        H2= self.pixel(H[0],H[1],H[2],gama,teta,beta,escala)
        
        AZ2= self.pixel(AZ[0],AZ[1],AZ[2],gama,teta,beta,escala)
        BZ2= self.pixel(BZ[0],BZ[1],BZ[2],gama,teta,beta,escala)
        CZ2= self.pixel(CZ[0],CZ[1],CZ[2],gama,teta,beta,escala)
        DZ2= self.pixel(DZ[0],DZ[1],DZ[2],gama,teta,beta,escala)
        EZ2= self.pixel(EZ[0],EZ[1],EZ[2],gama,teta,beta,escala)
        FZ2= self.pixel(FZ[0],FZ[1],FZ[2],gama,teta,beta,escala)
        GZ2= self.pixel(GZ[0],GZ[1],GZ[2],gama,teta,beta,escala)
        HZ2= self.pixel(HZ[0],HZ[1],HZ[2],gama,teta,beta,escala)
        
        #Dibujamos
        pygame.draw.polygon(screen,self.BLACK,(A2,B2,C2,D2)) 
        pygame.draw.polygon(screen,self.BLACK,(AZ2,BZ2,CZ2,DZ2))
        pygame.draw.polygon(screen,self.BLACK,(A2,B2,BZ2,AZ2))
        pygame.draw.polygon(screen,self.BLACK,(C2,D2,DZ2,CZ2))
        pygame.draw.polygon(screen,self.BLACK,(A2,D2,DZ2,AZ2))
        pygame.draw.polygon(screen,self.BLACK,(B2,C2,CZ2,BZ2))
        
        
        pygame.draw.polygon(screen,self.BLACK,(E2,F2,G2,H2))
        pygame.draw.polygon(screen,self.BLACK,(EZ2,FZ2,GZ2,HZ2))
        pygame.draw.polygon(screen,self.BLACK,(E2,F2,FZ2,EZ2))
        pygame.draw.polygon(screen,self.BLACK,(H2,G2,GZ2,HZ2))
        

        #x2=(x*math.cos(teta) - y*math.sin(teta))
        #y2=(y*math.cos(teta) + x*math.sin(teta))
        #z2=z
        
        return 0;
    def eslabon_q2 (self,screen,gama,teta,beta,escala,q1,q2):
        L1=10.48
        L2=16.43
        L3=9.27
        L4=0
        #Puntos en el espacio que dibujan el brazo
        A=self.cd_eslabon2(0,1,1,L1,L2,q1,q2)   #BASTAGO
        B=self.cd_eslabon2(0,-1,1,L1,L2,q1,q2)
        C=self.cd_eslabon2(0,-1,-1,L1,L2,q1,q2)
        D=self.cd_eslabon2(0,1,-1,L1,L2,q1,q2)
        
        AX=self.cd_eslabon2(-15,1,1,L1,L2,q1,q2)   #FIN 
        BX=self.cd_eslabon2(-15,-1,1,L1,L2,q1,q2)
        CX=self.cd_eslabon2(-15,-1,-1,L1,L2,q1,q2)
        DX=self.cd_eslabon2(-15,1,-1,L1,L2,q1,q2)
        
        #convertimos los puntos a R2
        A2= self.pixel(A[0],A[1],A[2],gama,teta,beta,escala)
        B2= self.pixel(B[0],B[1],B[2],gama,teta,beta,escala)
        C2= self.pixel(C[0],C[1],C[2],gama,teta,beta,escala)
        D2= self.pixel(D[0],D[1],D[2],gama,teta,beta,escala)
        
        A2X= self.pixel(AX[0],AX[1],AX[2],gama,teta,beta,escala)
        B2X= self.pixel(BX[0],BX[1],BX[2],gama,teta,beta,escala)
        C2X= self.pixel(CX[0],CX[1],CX[2],gama,teta,beta,escala)
        D2X= self.pixel(DX[0],DX[1],DX[2],gama,teta,beta,escala)
        
        #los dibujamos
        pygame.draw.circle(screen, self.RED2, (A2) , 3)
        pygame.draw.circle(screen, self.RED2, (B2) , 3)
        pygame.draw.circle(screen, self.BLUE, (C2) , 3)
        pygame.draw.circle(screen, self.BLUE, (D2) , 3)

        pygame.draw.circle(screen, self.RED2, (A2X) , 3)
        pygame.draw.circle(screen, self.RED2, (B2X) , 3)
        pygame.draw.circle(screen, self.BLUE, (C2X) , 3)
        pygame.draw.circle(screen, self.BLUE, (D2X) , 3)
        
        #dibujamos poligonos
        pygame.draw.polygon(screen,self.YELLOW,(A2,B2,C2,D2))
        pygame.draw.polygon(screen,self.YELLOW,(A2X,B2X,C2X,D2X))
        
        pygame.draw.polygon(screen,self.YELLOW2,(A2,B2,B2X,A2X))
        pygame.draw.polygon(screen,self.YELLOW2,(B2,C2,C2X,B2X))
        
        pygame.draw.polygon(screen,self.YELLOW2,(C2,D2,D2X,C2X))
        pygame.draw.polygon(screen,self.YELLOW,(D2,A2,A2X,D2X))
        #DIBUJAMOS UNION ESFÉRICA
        O=self.pixel(0,0,L1,gama,teta,beta,escala)
        pygame.draw.circle(screen, self.BLACK, (O) , 16)
        #DIBUJAMOS UNION ESFÉRICA FINAL
        #p=self.cd_eslabon2(0,0,0,L1,L2,q1,q2)
        #P=self.pixel(p[0],p[1],p[2],gama,teta,beta,escala)
        #pygame.draw.circle(screen, BLACK, (P) , 16)
        
    def eslabon_q3 (self,screen,gama,teta,beta,escala,q1,q2,q3):
        L1=10.48
        L2=16.43
        L3=9.27
        L4=0

        #Gripper
        E=self.cd_eslabon4(0,3,.7,L1,L2,L3,L4,q1,q2,q3,0)   #BASTAGO
        F=self.cd_eslabon4(0,-3,.7,L1,L2,L3,L4,q1,q2,q3,0)
        G=self.cd_eslabon4(0,-3,-.7,L1,L2,L3,L4,q1,q2,q3,0)
        H=self.cd_eslabon4(0,3,-.7,L1,L2,L3,L4,q1,q2,q3,0)
        
        I=self.cd_eslabon4(3,-3,-.7,L1,L2,L3,L4,q1,q2,q3,0)
        J=self.cd_eslabon4(3,3,-.7,L1,L2,L3,L4,q1,q2,q3,0)
        
        #Puntos en el espacio que dibujan el brazo
        A=self.cd_eslabon3(0,1,1,L1,L2,L3,q1,q2,q3)   #BASTAGO
        B=self.cd_eslabon3(0,-1,1,L1,L2,L3,q1,q2,q3)
        C=self.cd_eslabon3(0,-1,-1,L1,L2,L3,q1,q2,q3)
        D=self.cd_eslabon3(0,1,-1,L1,L2,L3,q1,q2,q3)
        
        O=self.cd_eslabon3(0,0,0,L1,L2,L3,q1,q2,q3)
        P=self.cd_eslabon3(-9.27,0,0,L1,L2,L3,q1,q2,q3)
        
        AX=self.cd_eslabon3(-9,1,1,L1,L2,L3,q1,q2,q3)   #FIN 
        BX=self.cd_eslabon3(-9,-1,1,L1,L2,L3,q1,q2,q3)
        CX=self.cd_eslabon3(-9,-1,-1,L1,L2,L3,q1,q2,q3)
        DX=self.cd_eslabon3(-9,1,-1,L1,L2,L3,q1,q2,q3)
        
        #convertimos los puntos a R2
        A2= self.pixel(A[0],A[1],A[2],gama,teta,beta,escala)
        B2= self.pixel(B[0],B[1],B[2],gama,teta,beta,escala)
        C2= self.pixel(C[0],C[1],C[2],gama,teta,beta,escala)
        D2= self.pixel(D[0],D[1],D[2],gama,teta,beta,escala)

        E2= self.pixel(E[0],E[1],E[2],gama,teta,beta,escala)
        F2= self.pixel(F[0],F[1],F[2],gama,teta,beta,escala)
        G2= self.pixel(G[0],G[1],G[2],gama,teta,beta,escala)
        H2= self.pixel(H[0],H[1],H[2],gama,teta,beta,escala)
        
        I2= self.pixel(I[0],I[1],I[2],gama,teta,beta,escala)
        J2= self.pixel(J[0],J[1],J[2],gama,teta,beta,escala)
        
        O2= self.pixel(O[0],O[1],O[2],gama,teta,beta,escala)
        P2= self.pixel(P[0],P[1],P[2],gama,teta,beta,escala)
        
        A2X= self.pixel(AX[0],AX[1],AX[2],gama,teta,beta,escala)
        B2X= self.pixel(BX[0],BX[1],BX[2],gama,teta,beta,escala)
        C2X= self.pixel(CX[0],CX[1],CX[2],gama,teta,beta,escala)
        D2X= self.pixel(DX[0],DX[1],DX[2],gama,teta,beta,escala)
        
        #los dibujamos
        pygame.draw.circle(screen, self.RED2, (A2) , 3)
        pygame.draw.circle(screen, self.RED2, (B2) , 3)
        pygame.draw.circle(screen, self.BLUE, (C2) , 3)
        pygame.draw.circle(screen, self.BLUE, (D2) , 3)

        pygame.draw.circle(screen, self.RED2, (A2X) , 3)
        pygame.draw.circle(screen, self.RED2, (B2X) , 3)
        pygame.draw.circle(screen, self.BLUE, (C2X) , 3)
        pygame.draw.circle(screen, self.BLUE, (D2X) , 3)
        
        #dibujamos poligonos
        pygame.draw.polygon(screen,self.YELLOW,(A2,B2,C2,D2))
        pygame.draw.polygon(screen,self.YELLOW,(A2X,B2X,C2X,D2X))
        
        pygame.draw.polygon(screen,self.YELLOW2,(A2,B2,B2X,A2X))
        pygame.draw.polygon(screen,self.YELLOW2,(B2,C2,C2X,B2X))
        
        pygame.draw.polygon(screen,self.YELLOW2,(C2,D2,D2X,C2X))
        pygame.draw.polygon(screen,self.YELLOW,(D2,A2,A2X,D2X))
        #DIBUJAMOS UNION ESFÉRICA
        pygame.draw.circle(screen, self.BLACK, (O2) , 10)
        #DIBUJAMOS UNION ESFÉRICA FINAL
        pygame.draw.circle(screen, self.BLACK, (P2) , 12)

        pygame.draw.polygon(screen,self.RED,(E2,F2,G2,H2))
        pygame.draw.polygon(screen,self.RED2,(H2,J2,E2))
        pygame.draw.polygon(screen,self.RED2,(F2,G2,I2))
    

def main(args=None):
    rclpy.init(args=args)
    interface_node = interface()

    interface_node.interface()
    
    rclpy.spin(interface_node)
    
    interface.destroy_node()
    rclpy.shutdown()
    
if __name__== "__main__":
    main()