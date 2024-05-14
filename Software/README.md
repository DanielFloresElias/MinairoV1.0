![](IMG/Minairó_Front_transp.png)
# MINAIRÓ V1.0 Robot
El robot Minairó es un proyecto *opensource* de robot didáctico, diseñado para aprender a programar de una manera divertida, proporcionando una plataforma *hardware* capaz de intreractuar mediante sensores y actuadores electrónicos.

Para manejar al MINAIRÓ se ha desarollado la libreria **Minairo_TCP.py**. En esta libreria encontramos los métodos necesarios para la comunicación mediante un socket TCP/IP.


### Métodos para comunicaciones: [->](#item1)
```
- MinairoSocket(IP,port)
- connect():
- run():
- transmit():
- stop():
- close():
- setPullingTime(value):
- getPullingTime():
```
### Métodos para control:
```
- setVel(x,y,w):
- setX(x):
- setY(y):
- setW(w):
```
### Métodos para el seguidor de líneas:
```
- setSensorLine_Threshold(value):
- getSensorLine_Threshold():
- getSensorLine_Analog():
- getSensorLine_Digital():
```
### Método para los sensores perimetrales:
```
- getSensorSharp():
```
### Métodos para las GPIOs:
```
- confGPIO(pin,mode):
- setGPIO(pin,value):
- getGPIO():
```
### Método para entradas analógicas:
```
getAnalogs()
```
### Método para servos RC:
```
- setSERVO(pin,value):
```
### Método par SONAR:
```
- getSONAR():
```
---
---
<a name="item1"></a>
## Métodos para comunicaciones:
### `MinairoSocket(IP,port)`
#### Descripción
Crea una instancia del Robot MINAIRÓ. En ella se incluyen todos los métodos para controlar el robot y monitorizar todos los sensores.

#### Sintaxis

```
MinairoSocket(IP,port)
```

#### Paràmetros
**IP**: Dirección IP del Robot MINAIRÓ. Por defecto 192.168.1.1.

**port**: Puerto para establecer el *socket* de comunicación. Por defecto *22*.

#### Ejemplo

```
from Minairo_TCP import *
from time import sleep

if __name__=="__main__":
    IP = '192.168.1.1'
    port = 22
    Robot = MinairoSocket(IP,port)
    Robot.run()
    Robot.setX(0.1)
    sleep(2)
    Robot.setX(0.0)
    sleep(0.5)
    Robot.close()
```
---
### `run()`
#### Descripción
El método *run()*, se conecta mediante el *socket* TCP/IP con el robot, e inicia la comunicación bidireccional ciclica entre el robot y su instancia de python.
El periodo de *polling* se establece mediante el método: *setPullingTime(value)* por defecto 25ms.

#### Sintaxis

```
run()
```

#### Paràmetros
Sin parámetros.

#### Ejemplo

```
from Minairo_TCP import *
from time import sleep

if __name__=="__main__":
    IP = '192.168.1.1'
    port = 22
    Robot = MinairoSocket(IP,port)
    Robot.run()
    Robot.setX(0.1)
    sleep(2)
    Robot.stop()
```
---
### `transmit()`
#### Descripción
El método *transmit()*, realiza una única comunicación bidireccional entre el robot y su instancia de python.En primer lugar transfiere el *OutBuffer* con todos los datos en *RAW* de control al MINAIRÓ, y seguidamente recibe el *InBuffer* con todos los datos en *RAW* procedentes de los sensores.

#### Sintaxis

```
transmit()
```

#### Paràmetros
Sin parámetros.

#### Ejemplo

```
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
    Robot.setX(0.0)
    Robot.transmit()
    Robot.close()
```

