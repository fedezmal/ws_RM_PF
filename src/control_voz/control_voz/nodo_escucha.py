import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient  # <--- NUEVO
from kobuki_ros_interfaces.msg import Sound
from nav2_msgs.action import NavigateToPose  # <--- NUEVO: La acción de Nav2
from geometry_msgs.msg import PoseStamped    # <--- NUEVO: Para las coordenadas

import threading
import pyaudio
import json
import os
import sys
from vosk import Model, KaldiRecognizer

class NodoEscuchaKobuki(Node):
    def __init__(self):
        super().__init__('nodo_escucha_kobuki')
        
        # --- CONFIGURACIÓN ---
        self.mic_index = None 
        self.trigger_word = "lazaro"
        
        # DICCIONARIO DE LUGARES [x, y]
        self.diccionario_lugares = {
            "cocina":  [1.46, -0.549],
            "puerta":  [0.0, 0.0],  # Origen suele ser 0,0
            "salón":   [5.0, -2.0],
            "salon":   [5.0, -2.0],
            "baño":    [3.0, 3.0]
        }
        # ---------------------

        # 1. Publicadores
        self.publisher_sound = self.create_publisher(Sound, '/commands/sound', 10)

        # 2. CLIENTE DE ACCIÓN PARA NAV2 (La integración nueva)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 3. Cargar Modelo Vosk
        self.get_logger().info("Cargando modelo de voz...")
        try:
            directorio_actual = os.path.dirname(os.path.abspath(__file__))
            ruta_modelo = os.path.join(directorio_actual, "model")
            if not os.path.exists(ruta_modelo):
                raise FileNotFoundError(f"No se encuentra modelo en: {ruta_modelo}")
            self.model = Model(ruta_modelo)
        except Exception as e:
            self.get_logger().error(f"Error crítico cargando modelo: {e}")
            sys.exit(1)

        self.get_logger().info("Modelo cargado.")

        # 4. Hilo de escucha
        self.stop_threads = False
        self.thread = threading.Thread(target=self.loop_escucha)
        self.thread.daemon = True
        self.thread.start()

    def loop_escucha(self):
        rec = KaldiRecognizer(self.model, 16000)
        p = pyaudio.PyAudio()
        
        try:
            stream = p.open(format=pyaudio.paInt16, 
                            channels=1, 
                            rate=16000, 
                            input=True, 
                            input_device_index=self.mic_index,
                            frames_per_buffer=4000)
            
            stream.start_stream()
            self.get_logger().info(f"Escuchando... Di '{self.trigger_word}' o 'llévame a...'")

            while not self.stop_threads:
                data = stream.read(4000, exception_on_overflow=False)
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    texto = result['text']
                    if texto:
                        self.get_logger().info(f"Oído: '{texto}'")
                        self.procesar_orden(texto)
        except Exception as e:
            self.get_logger().error(f"Error audio: {e}")
        finally:
            if 'stream' in locals():
                stream.stop_stream()
                stream.close()
            p.terminate()

    def procesar_orden(self, texto):
        texto = texto.lower()

        # COMANDO DE NAVEGACIÓN
        if "llevame" in texto or "llévame" in texto:
            destino_encontrado = None
            for lugar in self.diccionario_lugares:
                if lugar in texto:
                    destino_encontrado = lugar
                    break
            
            if destino_encontrado:
                coords = self.diccionario_lugares[destino_encontrado]
                # Llamamos a la nueva función integrada con Nav2
                self.iniciar_navegacion(destino_encontrado, coords)
            else:
                self.get_logger().warn("Lugar no reconocido.")
                os.system('espeak -v es "No conozco ese lugar" &')

        # ACTIVACIÓN
        elif self.trigger_word in texto or "lázaro" in texto:
            self.activar_robot()
        
        # DESPEDIDA
        elif "adios" in texto or "adiós" in texto:
            os.system('espeak -v es "Hasta luego" &') 

    def iniciar_navegacion(self, lugar, coordenadas):
        """
        Envía el Goal a Nav2 usando las coordenadas del diccionario.
        """
        x, y = coordenadas
        self.get_logger().info(f"🚀 [Nav2] Iniciando navegación hacia {lugar} ({x}, {y})")

        # 1. Feedback auditivo inmediato
        os.system(f'espeak -v es "Yendo a {lugar}" &')

        # 2. Comprobar si Nav2 está activo (wait_for_server)
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("¡Nav2 no responde! ¿Está lanzado el stack de navegación?")
            os.system('espeak -v es "Error, navegación no disponible" &')
            return

        # 3. Construcción del Goal (Adaptado de tu ejemplo)
        goal_msg = NavigateToPose.Goal()
        
        # Rellenamos el PoseStamped
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientación (w=1.0 significa mirando hacia "adelante" o neutro)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # 4. Enviar el objetivo
        self.get_logger().info(f"Enviando Goal a Nav2...")
        
        # Usamos send_goal_async para no bloquear el hilo de voz
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback cuando Nav2 acepta o rechaza la petición"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rechazado por Nav2 :(')
            return

        self.get_logger().info('Goal aceptado, navegando...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback cuando el robot llega al destino"""
        result = future.result().result
        self.get_logger().info('¡He llegado al destino!')
        os.system('espeak -v es "He llegado" &')

    def feedback_callback(self, feedback_msg):
        """Callback periódico con la distancia restante (opcional)"""
        feedback = feedback_msg.feedback
        # Nota: Imprimir esto puede ensuciar mucho el log, descomenta si quieres ver la distancia
        # print(f"Distancia restante: {feedback.distance_remaining:.2f} m")

    def activar_robot(self):
        self.get_logger().info('¡ACTIVADO!')
        sonido = Sound()
        sonido.value = 6 
        self.publisher_sound.publish(sonido)
        os.system('espeak -v es "Dime" &') 

    def cleanup(self):
        self.stop_threads = True
        self.thread.join()

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoEscuchaKobuki()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cleanup()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()