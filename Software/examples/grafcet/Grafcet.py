from tkinter import *
from Minairo_TCP import *

def Loop():
    # FUNCIÓN CICLICA
    # Esta función sé ejecuta ciclicamente con un periodo de 1 milisegundo.
    # mediante los objetos de tipo Clock de la libreria 'Minairo_TCP_V3_1', se podran ejecutar procesos en
    # intervalos de tiempo predeterminados. 

    # Definición de variables globales
    global CycleClock
    global Grafcet
    global T1
    global counter
    T1.update()
    #Definición de variables locales 

    if CycleClock.timeout():
            #############################################################
            ############# C O D I G O   D E   U S U A R I O #############
            #############################################################
            
            match Grafcet:

                 case 0:
                    Robot.setX(0.1)
                    Robot.setW(0.0)
                    T1.PT(10000)
                    T1.IN(True)
                    
                    if T1.Q():
                        T1.IN(False)
                        Grafcet = 10

                 case 10:
                    Robot.setX(0.0)
                    Robot.setW(0.6)
                    T1.PT(5000)
                    T1.IN(True)
                    if T1.Q():
                        T1.IN(False)
                        Grafcet = 0
                 case _:
                    Grafcet=0

            #############################################################
            ################# F I N   D E   C O D I G O #################
            #############################################################

    window.after(1,Loop) # Llamada programada en 1ms a la función Loop

def Sortir():
    Robot.stop()
    window.destroy()

if __name__=="__main__":
    IP = '192.168.1.1'
    Robot = MinairoSocket(IP,22)
    Grafcet = 0
    T1 = TON()
    counter = 0
    CycleClock = Clock(50)      # Reloj para ciclo de trabajo, periodo T=50ms F=20Hz.

    ## Creación i configuración de la ventana con Tkinter
    window = Tk()
    window.title("Minairo 1.0 -> IP{}".format(IP))
    window.config(width=220, height=200)

    ## Boto Exit
    btnSortir = Button(window, text="Exit", bg="grey",width=10,command=Sortir)
    btnSortir.place(x=70, y=150)

    ## Metodo para iniciar el socket de ethernet con el MINAIRÓ
    Robot.run()

    window.after(1,Loop)        # Llamada programada en 1ms a la función Loop
    window.mainloop()           # Arrancar la instancia de Ventana de Tkinter tipo Tk.
