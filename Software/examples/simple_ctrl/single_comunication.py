from Minairo_TCP import *
from time import sleep

if __name__=="__main__":
    IP = '192.168.1.1'
    port = 22
    Robot = MinairoSocket(IP,port)
    Robot.connect()
    Robot.setX(0.1)
    Robot.transmit()
    sleep(2)
    Robot.setX(0.2)
    Robot.transmit()
    Robot.close()