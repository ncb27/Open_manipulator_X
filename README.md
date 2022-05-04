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
Despues de esto se copian las posiciones el robot para hacer otro código basado en 
