# Open_manipulator_X
En este proyecto se trabajará en programar y simular a pequeña escala una implementación real de los brazos robóticos en la industria alimentaria en donde recientemente han tomado gran relevancia al volver más eficientes los procesos, incrementando así la producción y reduciendo los tiempos permitiendo también que los trabajadores ya no sean asignados a tareas repetitivas y puedan ser aprovechados en tareas con mayor relevancia.  
# requerimientos
Para poder replicar este proyecto se necesita lo sieguiente:
- conar con ubuntu 20.04
- instalar arduino como se muestra en: https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-linux hasta la parte de porting to Arduino.
- correr dentro de los ejemplos de arduino IDE: examples-OpenMnipulator-example-chain-open_manipulator_chain

![image](https://user-images.githubusercontent.com/99926615/166831342-cb22fd19-05ef-4bde-a473-13159d3f5eb1.png)
# funcionamiento
conectamos la trageta del robot a una fuente de luz, conectamos el usb a la computadora y conectamos el robot a la trageta OpenCR como se muestra en la siguiente imagen:

![image](https://user-images.githubusercontent.com/99926615/166833242-2f5832b0-6c9d-47a2-a4c2-a152b6f082fc.png)

una vez que tenemos este codigo cargado debe aparecer algo como lo siguiente:
![image](https://user-images.githubusercontent.com/99926615/166832047-efa71e64-f1f2-493b-826e-99862a2c0e4a.png)

lo cual nos indicará que el código se cargó correctamente en la trageta del robot.

Posteriormente se debe presionar el boton sw1 para iniciar el codigo precargado y el boton sw2 para iniciar el teaching mode, una vez inicializado el teaching mode el boton sw2 es para guardar la posisión donde se esta colocando el robot y el sw1 es para terminar con la programación.
Despues de esto se copian las posiciones el robot para hacer otro código basado en casos. Una vez teniendo las posiciones con sus respectivos arreglos, se procedió a modificar el switch case dependiendo de los movimientos individuales que se crearon para conformar toda la trayectoria, para nuestra aplicación, creamos 42 puntos, es por eso que en la sección de nuestro código, tenemos 42 cases. 
Una vez modificada esta sección, solo queda imprimir las instrucciones al usuario para que el manejo del robot sea sencillo de comprender, es por eso que, tenemos las siguientes instrucciones, la primera nos ayuda a indicar al usuario que para comenzar la trayectoria del robot es necesario presionar un botón en la tarjeta OpenCR.
![image](https://user-images.githubusercontent.com/99926615/166835798-f57b31b4-9b8b-4bad-b0da-24f6e69c7251.png)

Como segunda instrucción, tenemos una advertencia al usuario de que el robot comenzará a operar, esto es pensando en la escala real, ya que, en muchas ocasiones el operador no conoce el tiempo en el cual el robot comenzará a trabajar y pueden llegar a surgir percances o accidentes derivados de estos temas, es por ello que se decidió implementar este proceso de advertencia, el cual se puede ver a continuación:
![image](https://user-images.githubusercontent.com/99926615/166835917-79484503-c195-48af-9584-6e83b9a65f18.png)

Finalmente, se incluyen las instrucciones para parar o continuar el movimiento del robot, esto se pensó en caso de que haya algún error en la línea de producción, así como algún paro de emergencia al momento de realizar el pick and place, o alguna otra situación en la cual se necesite hacer un paro en el movimiento del robot. Así, el código que se muestra a continuación, nos muestra en el monitor serial las indicaciones para parar el movimiento, así como para empezarlo, una vez más, se utilizan los switches de la tarjeta OpenCR.
![image](https://user-images.githubusercontent.com/99926615/166836018-8b73c429-36ea-4ddf-af8e-61cc980454be.png)

Una vez terminado el código, es necesario cargar el programa a la tarjeta y comenzar a seguir las instrucciones que se nos indican en el monitor serial.
# resultado
en la siguiente imagen se muestra como acomodamos los bombones para que fueran paletizados en la charola y para colocar la tapadera.

