import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String

import pygame
from pygame.locals import *

import tkinter as tk
from tkinter import filedialog

from servicios.srv import ReproduceRoute

class Cell:
    """Crea objetos tipo casilla y los pinta en la pantalla de pygame.
    
    Args:
        size: Tamaño de la casilla
        color: Color con el cual se debe pintar la casilla
        row: Fila en que está ubicada la casilla dentro de la cuadrícula
        col: columna en que está ubicada la casilla dentro de la cuadrícula
    """
    def __init__(self,size,color,row,col):
        """ Constructor de la clase cell. 

        Args:
            size: Tamaño de la casilla en pixeles.
            color: Color con el cual se debe pintar la casilla
            row(int): Fila en que está ubicada la casilla dentro de la cuadrícula
            col: columna en que está ubicada la casilla dentro de la cuadrícula
        """
        self.size = size
        self.color = color
        self.row = row
        self.col = col

        #Calculamos las coordenadas (en pixeles) en las que se pintara la casilla:
        self.x = col*self.size
        self.y = row*self.size

    def paint(self,screen):
        """Pinta la casilla en la pantalla de pygame a partir de su posición en la cuadrícula y el tamaño en pixeles.
        
        Args:
            screen: pantalla en la cual se quiere pintar la casilla
        """
        pygame.draw.rect(screen,self.color,(self.x,self.y,self.size,self.size))

class Circle:
    """Crea objetos de tipo círculo y los dibuja en la pantalla de pygame.
    
    Args:
        center: indica las coordenadas (x,y) donde se desea centrar el círculo
        radius: radio del círculo
        color: color (RBG) con el cual se desea dibujar el círculo
    """
    def __init__(self,center,radius,color):
        """ Constructor de la clase circle. 
        
        Args:
            center: indica las coordenadas (x,y) donde se desea centrar el círculo
            radius: radio del círculo
            color: color con el cual se desea dibujar el círculo
        """
        self.center = center
        self.radius = radius
        self.color = color

    def paint(self,screen):
        """ Dibuja un círculo en la pantalla de pygame.
        
        Args:
            screen: pantalla en la cual se quiere pintar la el círculo
        """
        pygame.draw.circle(screen,self.color,self.center,self.radius,self.radius)

class Plant(Circle):
    """Crea objetos de tipo planta que heredan de la superclase círculo.
    
    Args: 
        center: indica las coordenadas (x,y) donde se desea centrar la planta
        radius: radio de la planta
        color: color (RBG) con el cual se desea dibujar la planta
    """
    def __init__(self, center, radius, color):
        """Constructor de la clase plant.
        
        Args: 
            center: indica las coordenadas (x,y) donde se desea centrar la planta
            radius: radio de la planta
            color: color (RBG) con el cual se desea dibujar la planta
        """
        super().__init__(center, radius, color)

class Robot(Circle):
    """Crea objetos de tipo robot que heredan de la superclase círculo.
    
    Args: 
        center: indica las coordenadas (x,y) donde se desea centrar el robot
        radius: radio del robot
        color: color (RBG) con el cual se desea dibujar la el robot
    """
    def __init__(self, center, radius, color):
        """Constructor de la clase robot.
        
        Args: 
            center: indica las coordenadas (x,y) donde se desea centrar el robot
            radius: radio del robot
            color: color (RBG) con el cual se desea dibujar el robot
        """
        super().__init__(center, radius, color)

    def move(self,x,y):
        """El robot se mueve a las coordenadas (x,y) indicadas.
        
        Args:
            x: Posición x, en el marco de referencia de coppelia, a la cual se mueve el robot
            y: posición y, en el marco de referencia de coppelia, a la cual se mueve el robot
        """
        xp = x*60+300 # Hallamos la equivalencia de la posición x en coppelia con los pixeles de la pantalla de pygame
        yp = -1*y*60+300 # Hallamos la equivalencia de la posición x en coppelia con los pixeles de la pantalla de pygame

        self.center = (xp,yp)

class Board:
    """Crea objetos de tipo tablero y los pinta en la pantalla de pygame. Este objeto está compuesto de una cuadrícula (grid), 
    una lista de plantas, un robot y un camino (path) que deja el robot al desplazarse.
    
    Args:
        dimension: Cantidad de casillas por fila (y columnas) de la cuadrícula (ej: cuadricula de 8x8 dimension = 8)
        size: Tamaño de la cuadricula en pixeles
        color1: color(RGB) con el cual se desea pintar las casillas impares de la cuadricula
        color2: color(RGB) con el cual se desea pintar las casillas pares de la cuadricula
        plants: Lista de objetos tipo planta
        robot: Objeto de tipo robot
    """
    def __init__(self,dimension,size,color1,color2,plants,robot):
        """Constructor de la clase grid.
        
        Args:
            dimension: Cantidad de casillas por fila (o columna) de la cuadricula (ej: cuadricula de 8x8 dimension = 8)
            size: Tamaño de la cuadricula en pixeles
            color1: color(RGB) con el cual se desea pintar las casillas impares de la cuadricula
            color2: color(RGB) con el cual se desea pintar las casillas pares de la cuadricula
            plants: Lista de objetos tipo planta
            robot: Objeto de tipo robot
        """
        self.dimension = dimension
        self.size = size
        self.color1 = color1
        self.color2 = color2
        self.grid = self.createGrid()
        self.plants = plants
        self.robot = robot

        #Creamos la variable path que guarda las diferentes ubicaciones del robot para poder pintar su recorrido
        self.path = []


    def createGrid(self):
        """Crea un arreglo bidimensional de celdas según la dimensión del tablero.
        
        Returns: 
            grid: Arreglo que contiene los objetos de tipo casilla que compondrán el tablero
        """
        cellsize = self.size/self.dimension #Determinamos el tamaño de la celda a partir del tamaño del tablero y las dimensiones deseadas
        grid = [] #Variable en la cual se va a guardar el arreglo

        for y in range(self.dimension):
            row = []
            for x in range(self.dimension):
                if (x + y)% 2 == 0: #Sacamos el modulo para determinar si es una casilla par o impar y asi pintarla de un color determinado
                    cell = Cell(cellsize,self.color2,y,x)
                else: 
                    cell = Cell(cellsize,self.color1,y,x)
                row.append(cell)
            grid.append(row)
        return grid

    def paintPath(self,screen):
        """Pinta el camino que ha recorrido el robot sobre la pantalla de pygame.

        Args: 
            screen: pantalla en la cual se quiere pintar el camino recorrido
        """
        for i in range(1,len(self.path)): #Recorremos la lista del recorrido del robot y la dibujamos en la pantalla de pygame
            pygame.draw.line(screen,self.robot.color, self.path[i-1], self.path[i])
        
    def paint(self,screen):
        """Pinta todo el escenario compuesto por el grid, las plantas, el robot y el recorrido del robot en la pantalla de pygame.
        
        Args:
            screen: Pantalla en la cual se quiere pintar el tablero
        """
        self.screen = screen

        for i in range(self.dimension): #Recorremos el arreglo de casillas para pintar el tablero
            for j in range(self.dimension):
                self.grid[i][j].paint(screen)

        for i in range(len(self.plants)): #Recorremos la lista de plantas y pintamos cada una de ellas sobre el tablero
            self.plants[i].paint(screen)

        self.robot.paint(screen) #Pintamos el robot
        self.paintPath(screen) #Pintamos su recorrido

    def moveRobot(self, msgx, msgy):
        """Mueve el robot a la posición (x,y) contenida en msg.

        Args:
            msgx: contiene la coordenada x de la ubicación del robot contenidas en el mensaje recibido a través del tópico 'turtlebot_position'
            msgy: contiene la coordenada y de la ubicación del robot contenidas en el mensaje recibido a través del tópico 'turtlebot_position'
        """
        self.robot.move(msgx,msgy)
        self.path.append(self.robot.center)
        self.paint(self.screen)

    def saveImage(self, img_name):
        """Guarda la imagen del tablero en la ruta deseada con el nombre indicado por el usuario.
        
        Args:
            img_name: Nombre con el cual el usuario quiere guardar la imagen
        """
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.asksaveasfilename(initialfile = img_name, defaultextension=".jpg",
                                filetypes=[("Archivo JPG", "*.jpg")])
        if file_path:
            # Save the image to the selected file path
            pygame.image.save(self.screen, file_path)
            img = pygame.image.load(file_path)
            tablero = pygame.Rect(0,0,500,500)
            tab_recortado = img.subsurface(tablero)
            pygame.image.save(tab_recortado, file_path)
        root.destroy()

class TextBox():
    """Crea objetos de tipo 'caja de texto' con un tipo de letra, color y ubicación.
    
    Args:
        text: texto que se quiere escribir en la caja de texto
        fuente: tipo de letra que se va a utilizar
        color: color del texto que se va a escribir
        center: ubicación de la caja de texto en la pantalla de pygame (pixeles)
    """
    def __init__(self, text, fuente, color, center):
        """Constructor de la clase 'TextBox'.
        Args:
            text: texto que se quiere escribir en la caja de texto
            fuente: tipo de letra que se va a utilizar
            color: color del texto que se va a escribir
            center: ubicación de la caja de texto en la pantalla de pygame (pixeles)
        """
        self.fuente = fuente
        self.color = color
        self.center = center
        self.setText(text)

    def setText(self, text):
        """Crea un cuadro de texto con el texto deseado.

        Args:
            text: texto que se va a añadir en el cuadro de texto
        """
        self.text = text
        self.surface = self.fuente.render(text, True, self.color)
        self.box = self.surface.get_rect()
        self.box.center = self.center

    def paint(self, screen):
        """Pinta el cuadro de texto en la pantalla de pygame.
        
        Args:
            screen: pantalla en la cual se desea pintar el cuadro de texto
        """
        screen.blit(self.surface, self.box)

class Button():
    """Crea objetos tipo 'Botón' con el caption deseado.
    
    Args:
        rectangle: rectángulo en el cual se va a contener el botón
        fuente: tipo de letra a utilizar para escribir el caption del botón
        backgroundColor: color de fondo con el cual se va a pintar el botón
        textColor: color del texto con el que se escribe el caption del botón
        caption: texto a escribir en el botón
    """
    def __init__(self, rectangle, fuente, backgroundColor, textColor,caption):
        """Constructor de la clase botón.
    
        Args:
            rectangle: rectángulo en el cual se va a contener el botón
            fuente: tipo de letra a utilizar para escribir el caption del botón
            backgroundColor: color de fondo con el cual se va a pintar el botón
            textColor: color del texto con el que se escribe el caption del botón
            caption: texto a escribir en el botón
        """
        self.rectangle = rectangle
        self.color = backgroundColor
        self.caption = caption
        self.surface = fuente.render(self.caption, True, textColor)
        self.box = self.surface.get_rect()
        self.box.center = self.rectangle.center

    def paint(self, screen):
        """Pinta el botón en la ventana de pygame.
        
        Args: 
            screen: Pantalla en la cual se desea pintar el botón
        """
        pygame.draw.rect(screen, self.color, self.rectangle)
        screen.blit(self.surface, self.box)  

class DialogBox():
    """Crea objetos de tipo cuadro de dialogo.
    
    Args:
        screen: pantalla en la cual se va a pintar el cuadro de dialogo
        buttons: lista de los botones a añadir en el cuadro de dialogo
        instructionsTextBox: caja de texto que contiene las instrucciones de la ventana
        nameTextBox: caja de texto en la cual el usuario puede escribir en la interfaz
    """
    def __init__(self, screen, buttons, instructionsTextBox, nameTextBox=None):
        """Constructor de la clase cuadro de dialogo.
    
        Args:
            screen: pantalla en la cual se va a pintar el cuadro de dialogo
            buttons: lista de los botones a añadir en el cuadro de dialogo
            instructionsTextBox: caja de texto que contiene las instrucciones de la ventana
            nameTextBox: caja de texto en la cual el usuario puede escribir en la interfaz
        """
        self.screen = screen
        self.buttons = buttons
        self.instructionsTextBox = instructionsTextBox
        self.nameTextBox = nameTextBox

    def eventListener(self,window):
        """Escucha los eventos en los cuadros de dialogo, actua cuando se presiona algún botón.
        
        Args:
            window: string que indica la ventana actual
        """

        running = True
        selection = ""

        if window == "Menu": #Ventana Inicial
            win = 5
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    for i in range(len(self.buttons)): #Recorremos la lista de botones y pintamos cada uno sobre la pantalla
                        if self.buttons[i].rectangle.collidepoint(event.pos):
                            win = i
                            print(f"Boton {i}")
                            break

            #Evalúa el botón presionado                
            if win == 0:
                running = False
                selection = "Teclas"
            elif win == 1:
                running = False
                selection = "TXT"
        
        elif window == "Teclas":
            win = 5
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    for i in range(len(self.buttons)): #Recorremos la lista de botones y pintamos cada uno sobre la pantalla
                        if self.buttons[i].rectangle.collidepoint(event.pos):
                            win = i
                            break
            if win == 0:
                running = False
                selection = "Yes"
            elif win == 1:
                running = False
                selection = "No"

        elif window == "TXT":
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    for i in range(len(self.buttons)): #Recorremos la lista de botones y pintamos cada uno sobre la pantalla
                        if self.buttons[i].rectangle.collidepoint(event.pos):
                            running = False
                            selection = "BuscadorArchivos"
                            print(f"Se presiono boton {selection}")
                            break

        elif window == "Tablero":
            img_name = self.nameTextBox.text
            selection = ""
            win = 5
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_BACKSPACE:
                        img_name = img_name[:-1]
                    else:
                        img_name += event.unicode
                elif event.type == pygame.MOUSEBUTTONUP:
                    for i in range(len(self.buttons)): #Recorremos la lista de botones y pintamos cada uno sobre la pantalla
                        if self.buttons[i].rectangle.collidepoint(event.pos):
                            win = i
                            break
    
            if win == 0:
                selection = "Save"
                root = tk.Tk()
                root.withdraw()
                file_path = filedialog.asksaveasfilename(
                    initialfile = self.nameTextBox.text, 
                    defaultextension=".jpg", 
                    filetypes=[("Archivo JPG", "*.jpg")]
                )
                if file_path:
                    # Save the image to the selected file path
                    pygame.image.save(self.screen, file_path)
                    img = pygame.image.load(file_path)
                    tablero = pygame.Rect(0,0,500,500)
                    tab_recortado = img.subsurface(tablero)
                    pygame.image.save(tab_recortado, file_path)
                root.destroy()
            elif win == 1:
                selection = "end"
                        
            self.nameTextBox.setText(img_name)
            self.paintDialog((250,250,250),(0,500, 500, 80))

        return running, selection

    def paintDialog(self,bgColor,rect):
        """Pinta el cuadro de dialogo en la ventana de pygame.
        
        Args:
            bgColor: color de fondo para el cuadro de dialogo
            rect: rectangulo en el que se pinta el cuadro de dialogo dentro de la interfaz
        """
        pygame.draw.rect(self.screen, bgColor,rect)

        if self.nameTextBox != None:
            self.nameTextBox.paint(self.screen)

        self.instructionsTextBox.paint(self.screen)

        for i in range(len(self.buttons)): #Recorremos la lista de botones y pintamos cada uno sobre la pantalla
            self.buttons[i].paint(self.screen)

class Interface(Node):
    """Crea la interfaz. Hereda de la superclase Node.

    Args:
        screen: pantalla en la que se va pintar
        board: tablero a pintar objeto de la clase Board 
        dialogbox: cuadro de dialogo a pintar objeto de la clase DialogBox
        window: string que indica la ventana actual
    """

    def __init__(self, screen, board, dialogBox,window):
        """Cosntructor de la clase interfaz. Crea el nodo 'turtle_bot_interface' y se suscribe al tópico 'turtlebot_position'
        para conocer la posición en tiempo real del robot sobre el entorno de coppelia.

        Args:
            screen: pantalla en la que se va pintar
            board: tablero a pintar objeto de la clase Board 
            dialogbox: cuadro de dialogo a pintar objeto de la clase DialogBox
            window: string que indica la ventana actual
        """

        super().__init__('turtle_bot_interface')

        #Creamos la subscripción al tópico 'turtlebot_position'
        self.subscriptionTwist = self.create_subscription(Twist,'turtlebot_position',self.listener_position,10)
        #Creamos la subscripción al tópico 'turtlebot_route'
        self.subscriptionString = self.create_subscription(String,'turtlebot_route',self.listener_route,10)

        #Creamos el cliente:
        self.client = self.create_client(ReproduceRoute, "/RP")

        self.screen = screen
        self.board = board
        self.dialogBox = dialogBox
        self.window = window
        self.route = []

    def CallClient_TxtRoute(self,file_path):
        """Llama al cliente ReproduceRoute y le hace una solicitud.
         
        Args:
            file_path: ruta del archivo TXT que se desea replicar con coppelia.
        """
        #self.client = self.create_client(ReproduceRoute, "/RP")
        print(f"[INFO] file_path = {file_path}")

        #if file_path != "":
        request = ReproduceRoute.Request()
        request.file_path = str(file_path)
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self,future=self.future)

        self.estado=self.future.result().ruta
        self.get_logger().info(self.estado) 
        self.get_logger().info('Funciona <3 <3 <3') 
        print(f"[INFO] Request route form txt sent. request = {request}")
            
    def listener_route(self, msg):
        """Escucha los mensajes tipo String() que se reciben por el tópico 'turtlebot_route' y los añade a una lista 'route'.
        
        Args:
            msg: mensaje enviado a través del tópico 'turtlebot_route'
        """
        self.route.append(msg.data+"\n")
    
    def SaveRoute(self):
        """Guarda la lista 'route' en un archivo .txt"""
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Archivo TXT","*.txt")]
        )
        with open(file_path,'w') as f:
            f.writelines(self.route) 

    def listener_position(self, msg):
        """Escucha los mensajes tipo Twist() que se reciben por el tópico 'turtlebot_position' y replica los movimientos del robot
        en la interfaz.
        
        Args:
            msg: mensaje tipo Twist que contiene la posición actual del robot en el marco de referencia inercial
        """
        msgx = msg.linear.x
        msgy = msg.linear.y

        '''Revisa que en el caso de tener un evento de tipo QUIT, es decir, que se presione la x en la ventana de
        pygame, esta se cierre cambiando la variable running a false'''
        running,selection = self.dialogBox.eventListener(self.window)

        if  running:
            '''Mueve el robot a la posición que viene de Coppelia'''        
            self.board.moveRobot(msgx,msgy)

            if selection == "end":
                self.SaveRoute()
                print(f"[INFO] Route Saved.")


            '''Actualiza la pantalla de pygame para que aparezca en pantalla el movimiento del robot'''
            pygame.display.flip()

        else:
            print("------> Tratando de destruir <-------")
            pygame.quit()
            self.destroy_node()
            rclpy.shutdown()

class App():
    """Crea objetos de tipo Aplicación (interfaz)."""

    def createDisplay(self, coordinate,caption):
        """Crea la pantalla en la cual se dibuja el recorrido del robot.
        
        Args:
            coordinate: dupla que contiene el tamaño de la pantalla que se quiere crear
            caption: nombre que se quiere mostrar en la ventana
        """
        screen = pygame.display.set_mode(coordinate)
        pygame.display.set_caption(caption)
        return screen
    
    def createMenuButtons(self, fuente, bgColor, TxtColor):
        """Crea el arreglo de botones del menu.
        
        Args:
            fuente: tipo de letra que se quiere usar para escribir el texto de los botones
            bgColor: color de fondo que se quiere usar para crear los botones
            TxtColor: color del texto que se quiere usar para escribir el texto de los botones
        """
        buttons = []
        buttons.append(Button(pygame.Rect(80, 100, 350, 50), fuente, bgColor, TxtColor,"Mover Coppelia con Teclas"))
        buttons.append(Button(pygame.Rect(80, 180, 350, 50), fuente, bgColor, TxtColor,"Mover Coppelia con Archivo .Txt"))
        
        return buttons
    
    def createYesNoButtons(self, fuente, bgColor, TxtColor):
        """Crea un arreglo con los botones 'Si' y 'No'.
        
        Args:
            fuente: tipo de letra que se quiere usar para escribir el texto de los botones
            bgColor: color de fondo que se quiere usar para crear los botones
            TxtColor: color del texto que se quiere usar para escribir el texto de los botones
        """
        buttons = []
        buttons.append(Button(pygame.Rect(200, 100, 100, 50), fuente, bgColor, TxtColor,"Si"))
        buttons.append(Button(pygame.Rect(200, 180, 100, 50), fuente, bgColor, TxtColor,"No"))
        
        return buttons

    def createSelectButton(self,fuente, bgColor, TxtColor):
        """Crea un arreglo con el botón 'seleccionar'.
        
        Args:
            fuente: tipo de letra que se quiere usar para escribir el texto de los botones
            bgColor: color de fondo que se quiere usar para crear los botones
            TxtColor: color del texto que se quiere usar para escribir el texto de los botones
        """
        buttons = []
        buttons.append(Button(pygame.Rect(100, 150, 250, 50), fuente, bgColor, TxtColor,"Seleccionar"))
        
        return buttons

    def createBoardButtons(self,fuente, bgColor, TxtColor,btn,color):
        """Crea un arreglo con el botón 'seleccionar'.
        
        Args:
            fuente: tipo de letra que se quiere usar para escribir el texto de los botones
            bgColor: color de fondo que se quiere usar para crear los botones
            TxtColor: color del texto que se quiere usar para escribir el texto de los botones
            btn: variable de tipo Boolean que indica si se debe añadir el botón 'Terminar Recorrido'
            color: color para pintar el botón 'Terminar Recorrido'
        """
        buttons = []
        buttons.append(Button(pygame.Rect(200, 550, 100, 25), fuente, bgColor, TxtColor,"Guardar"))
        #si se eligió guardar el recorrido se añade el botón de "Terminar recorrido"
        if btn:
            buttons.append(Button(pygame.Rect(300, 500, 200, 25), fuente, color, TxtColor,"Terminar Recorrido"))

        return buttons
    
    def createPlants(self, color):
        """Crea el arreglo de plantas y les asigna un color.
        
        Args:
            color: color (RGB) con el cual se desea pintar las plantas en la interfaz
        """
        plants = []
        
        return plants


    def createFont(self,type,size):
        """Crea una fuente del tipo y tamaño seleccionado.
        
        Args:
            type: tipo de letra a crear, puede ser negrilla o normal
            size: tamaño de la fuente a crear
        """
        if type == "bold":
            fuente = pygame.font.SysFont("Arial Black",size)
        elif type == "normal":
            fuente = pygame.font.SysFont("Arial",size)

        return fuente

    def run(self):
        """Ejecuta la aplicación."""
        #CONSTANTES:

        #Tamaño de la pantalla de pygame en pixeles:
        menu_screen_width = 500
        menu_screen_height = 300

        screen_width = 500
        screen_height = 580

        #Colores en RGB:
        White = (255,255,255)
        Black = (0,0,0)
        LigthGrey = (250, 250, 250)
        Grey = (205,205,205)
        Green = (0,143,57)
        Red = (255,0,0)

        #Tamaño del tablero:
        board_size = 500

        #Dimensiones de la cuadricula:
        dimension = 10

        #creamos las fuentes a utilizar:
        pygame.font.init()
        fuente = self.createFont("bold",20)
        fuente2 = self.createFont("normal",20)
        fuenteMenu = self.createFont("bold",30)

        # Crea la ventana del menú:
        caption = "Turtle Bot Menu"
        menu = self.createDisplay((menu_screen_width, menu_screen_height),caption)

        #creamos el cuadro de texto para dar indicaciones:
        InstructionsTextBox = TextBox("Bienvenido, elija una opción:", fuenteMenu, Black, (250, 50))

        #Creamos la lista de botones de la pantalla:
        buttons = self.createMenuButtons(fuenteMenu,Grey,Black)

        #Pintamos la escena en la pantalla de pygame creada:
        dialogBox = DialogBox(menu, buttons, InstructionsTextBox)
        dialogBox.paintDialog(White,(0,0, 500, 300))

        #Variables que indican el botón presionado en el menu:
        winTeclas = False
        winTXT = False
        winTablero = False
        winTablero2 = False

        #Variable que guarda la ventana actual:
        window = "Menu"

        # Ciclo principal del menu:
        running = True
        while running:

            pygame.display.flip()
            running,selection = dialogBox.eventListener(window)

            if selection == "Teclas":
                winTeclas = True
                window = selection
            elif selection =="TXT":
                winTXT = True
                window = selection

        #Guardamos la respuesta del usuario en un mensaje tipo Bool:
        answ = False
        file_path = ""

        if winTeclas:
            #Pantalla que pregunta si se quiere guardar el recorrido:
            caption = "Turtle Bot Menu"
            pregunta = self.createDisplay((menu_screen_width, menu_screen_height),caption)

            #creamos el cuadro de texto para dar indicaciones:
            InstructionsTextBox = TextBox("¿Desea Guardar el Recorrido?", fuenteMenu, Black, (250, 50))

            #Creamos la lista de botones de la pantalla:
            buttons = self.createYesNoButtons(fuenteMenu,Grey,Black)

            #Pintamos la escena en la pantalla de pygame creada:
            dialogBox = DialogBox(menu, buttons, InstructionsTextBox)
            dialogBox.paintDialog(White,(0,0, 500, 300))

            # Ciclo principal de la pantalla pregunta:
            running = True
            while running:

                pygame.display.flip()
                running,selection = dialogBox.eventListener(window)

                if selection == "Yes":
                    winTablero = True
                    window = "Tablero"
                    #Pasamos el resto de ventanas a falso:
                    winTeclas = False
                    winTXT = False
                    winTablero2 = False
                    answ = True
                elif selection == "No":
                    winTablero2 = True
                    window = "Tablero"
                    #Pasamos el resto de ventanas a falso:
                    winTeclas = False
                    winTXT = False
                    winTablero = False

        elif winTXT:
            #Pantalla que pregunta si se quiere guardar el recorrido:
            caption = "Turtle Bot Menu"
            pregunta = self.createDisplay((menu_screen_width, menu_screen_height),caption)

            #creamos el cuadro de texto para dar indicaciones:
            InstructionsTextBox = TextBox("Seleccione el archivo que contiene el recorrido", fuenteMenu, Black, (250, 50))

            #Creamos la lista de botones de la pantalla:
            buttons = self.createSelectButton(fuenteMenu,Grey,Black)

            #Pintamos la escena en la pantalla de pygame creada:
            dialogBox = DialogBox(menu, buttons, InstructionsTextBox)
            dialogBox.paintDialog(White,(0,0, 500, 300))

            # Ciclo principal de la pantalla pregunta:
            running = True
            while running:

                pygame.display.flip()
                running,selection = dialogBox.eventListener(window)

                if selection == "BuscadorArchivos":
                    winTablero2 = True
                    window = "Tablero"

                    #Pasamos el resto de ventanas a falso:
                    winTeclas = False
                    winTXT = False
                    winTablero = False

                    #print(f"Estamos abriendo el buscador de archivos")

                    #Abrimos el buscador de archivos para obtener la ruta del archivo:
                    root = tk.Tk()
                    root.withdraw()
                    file_path = filedialog.askopenfilename(
                        defaultextension=".txt", 
                        filetypes=[("Archivo TXT", "*.txt")]
                    )
                    print(f"File Path {file_path}")

        if winTablero or winTablero2:

            #Se crea una nueva pantalla:
            caption = "Turtle Bot Interface"
            screen = self.createDisplay((screen_width, screen_height),caption)

            #creamos el cuadro de texto para dar indicaciones:
            instructionsTextBox = TextBox("Nombre de Archivo:", fuente, Black, (80, 512))

            #creamos el cuadro de texto en el que se va a guardar el nombre de la imagen:
            nameTextBox = TextBox("", fuente2, Black, (250, 530))

            #Creamos los botones:
            if winTablero:
                button = self.createBoardButtons(fuente, Grey, Black,True,Red)
            else: 
                button = self.createBoardButtons(fuente, Grey, Black,False,Red)

            #Creamos la lista de plantas y añadimos 4 objetos de tipo planta que recreen las observadas en Coppelia:
            plants = self.createPlants(Green)

            #creamos el objeto tipo robot y lo inicializamos en el centro de la pantalla:
            robot = Robot((board_size/2, board_size/2), 20, Red)

            #creamos el onjeto tipo tablero:
            board = Board(dimension,board_size,White,Grey,plants,robot)

            #Pintamos la escena en la pantalla de pygame creada:
            board.paint(screen)

            dialogBox = DialogBox(screen, button, instructionsTextBox, nameTextBox)
            dialogBox.paintDialog(White,(0,500, 500, 80))

            #creamos el nodo 'interface_node':
            rclpy.init()
            interface_node = Interface(screen, board, dialogBox,window)
            
            #En el caso en que se elija replicar recorrido de archivo txt:
            if file_path != "":
                print(f"[INFO] calling client txt route {file_path}")
                interface_node.CallClient_TxtRoute(file_path)
                
            rclpy.spin(interface_node)

            pygame.quit()
            interface_node.destroy_node()
            rclpy.shutdown()

        

def main(args=None):
    app = App()
    pygame.init()
    app.run()

if __name__ == '__main__':
    main()