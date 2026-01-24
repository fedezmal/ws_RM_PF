**Instrucciones de Instalación**

- Para poder ejecutar el código hay que tener instalado pyaudio y vosk, en ubuntu hay que instalar el paquete portaudio19-dev antes de instalar pyaudio con pip, a no ser que se esté utilizando un docker, los paquetes deberían instalarse fuera de cualquier entorno virtual de python ya que ROS2 ejecuta los nodos con la instalación de python nativa, es posible que haya que utilizar el flag "--break-system-packages" a la hora de instalar los paquetes.

- Este proyecto está pensado como un nodo de ROS2, por lo que la instalación se realiza copiando los contenidos del repositorio directamente en la raiz del workspace personal de ROS del usuario y ejecutando "colcon build" en dicho directorio.

- Antes de poder ejecutar el nodo con "ros2 run control_voz escuchar" es necesario modificar el código base del nodo ubicado en "src/control_voz/control_voz/nodo_escucha.py". El ususario deberá indicar el puerto de su microfono en la linea 18: "self.mic_index = None" remplazando "None" por el índice de su microfono, así como modificar el número de canales en la linea 99 "stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000,". Para conocer los valores específicos de la máquina del usuario se pone a su disposición un script python check_mic.py.

**Instrucciones de uso**

El nodo está pensado para ser utilizado con robots kobuki turtlebot 3, por lo que se considera necesario tener instalado el paquete kobuki en el workspace de ROS. El nodo utiliza los topics /amcl_pose y /goal_pose, por lo que una vez iniciada la base del kobuki y el laser se debería iniciar el módulo de localización con "ros2 launch nav2_bringup localization_launch.py map:=mapa1.yaml" (los archivos mapa1.yaml y mapa1.pgm están disponibles en la carpeta "mapas" del repositorio, deberían estar los dos en la carpeta desde la que se ejecute el comando) y el modulo de navegación con "ros2 launch kobuki navigation.launch.py map:=mapa1.yaml".

Una vez hech esto, el nodo se puede iniciar con "ros2 run control_voz escuchar", si todo el stack de navegación estaba funcionando correctamente antes de arrancar el nodo todos los comandos deberían funcionar.

**Comandos disponibles**

En la versión actual el robot es capaz de guiar al usuario hasta una ubiación guardada en el mapa proporcionado al stack de navegación, añadir nuevos puntos e indidcar en que punto se encuentra. Los comandos son los siguientes:
  - "Añade *nombre de marcador*": añade un marcador con el nombre indicado en la posición actual del robot
  - "Llévame a *nombre de marcador*": guía al usuario hasta el marcador nombrado, buscandolo en su lista de marcadores y publicando las coordenadas en el topic /goal_pose
  - "Dónde estoy": indica el punto mas cercano, dentro de un margen admisible, al robot, diciendo "estámos en *nombre del marcador*".
