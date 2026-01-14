import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kobuki_ros_interfaces.msg import Sound # <--- NUEVO: Importamos el mensaje de sonido
import os

class NodoEscucha(Node):
    def __init__(self):
        super().__init__('nodo_escucha_simple')
        
        # Suscriptor para escuchar tu orden
        self.subscription = self.create_subscription(
            String,
            'orden_palabra',
            self.listener_callback,
            10)
            
        # NUEVO: Publicador para mandar sonido al robot
        self.publisher_sound = self.create_publisher(Sound, '/commands/sound', 10)
        
        self.get_logger().info('🐢 Nodo listo: Di "Lazaro" para hablar y pitar.')

    def listener_callback(self, msg):
        palabra = msg.data.lower().strip()
        self.get_logger().info(f'He escuchado: "{palabra}"')

        if palabra == 'Lazaro':
            # 1. El PC Habla
            self.get_logger().info('¡Palabra mágica! -> Hablando y Pitando')
            os.system('espeak "Hola, soy tu Turtlebot"')
            
            # 2. El Robot Pita
            sonido_msg = Sound()
            sonido_msg.value = 6  # El 6 es el sonido de "Power On" (muy audible)
            self.publisher_sound.publish(sonido_msg)
            
        elif palabra == 'adios':
            os.system('espeak "Hasta luego"')
        else:
            self.get_logger().info('Esa no es la palabra mágica...')

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoEscucha()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()