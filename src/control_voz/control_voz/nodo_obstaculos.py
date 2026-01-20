import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray
from kobuki_ros_interfaces.msg import Sound
import time
import os

class WatchdogPuerta(Node):
    def __init__(self):
        super().__init__('watchdog_puerta')
        
        # Suscripciones
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        # Nav2 publica el estado de sus acciones aquí (Jazzy/Humble)
        self.status_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10)
        
        self.sound_pub = self.create_publisher(Sound, '/commands/sound', 10)

        self.is_navigating = False
        self.last_move_time = time.time()
        self.stuck_threshold = 3.0 # Segundos quieto para considerar que "espera puerta"
        
        # Timer para chequear estado
        self.timer = self.create_timer(1.0, self.check_stuck)

    def status_callback(self, msg):
        # Chequeamos si hay alguna acción de navegación activa (Status 2=EXECUTING)
        if msg.status_list:
            estado = msg.status_list[-1].status
            self.is_navigating = (estado == 2)
        else:
            self.is_navigating = False

    def vel_callback(self, msg):
        # Si hay velocidad, actualizamos el reloj
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_move_time = time.time()

    def check_stuck(self):
        if self.is_navigating:
            time_stopped = time.time() - self.last_move_time
            
            # Si lleva quieto más de X seg y está navegando -> Asumimos bloqueo
            if time_stopped > self.stuck_threshold:
                self.get_logger().warn("🛑 ¡Bloqueado en puerta! Solicitando paso...")
                
                # 1. Pitar
                msg = Sound()
                msg.value = 6 
                self.sound_pub.publish(msg)
                
                # 2. Hablar (Opcional, no bloquear el hilo)
                os.system('espeak -v es "Permiso por favor" &')
                
                # Reiniciamos el contador para no pitar como loco cada segundo
                # Le damos 5 segundos de gracia (lo que dura el Wait del XML)
                self.last_move_time = time.time() + 2.0 

def main():
    rclpy.init()
    node = WatchdogPuerta()
    rclpy.spin(node)
    rclpy.shutdown()