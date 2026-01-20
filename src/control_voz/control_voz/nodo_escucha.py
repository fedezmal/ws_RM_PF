import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import Sound
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
        # Si tienes problemas, cambia None por el número de índice (ej. 8)
        self.mic_index = None 
        self.trigger_word = "lazaro"
        # ---------------------

        # 1. Publicador para el sonido del Kobuki
        self.publisher_sound = self.create_publisher(Sound, '/commands/sound', 10)
        
        # 2. Cargar Modelo Vosk
        self.get_logger().info("Cargando modelo de voz...")
        try:
            # Obtenemos la ruta del directorio donde está ESTE script ejecutándose
            directorio_actual = os.path.dirname(os.path.abspath(__file__))
            
            # Construimos la ruta al modelo asumiendo que está en la misma carpeta
            ruta_modelo = os.path.join(directorio_actual, "model")
            
            self.get_logger().info(f"Buscando modelo en: {ruta_modelo}")

            if not os.path.exists(ruta_modelo):
                raise FileNotFoundError(f"Carpeta 'model' no encontrada en {ruta_modelo}")
            
            self.model = Model(ruta_modelo)
        except Exception as e:
            self.get_logger().error(f"Error crítico cargando modelo: {e}")
            sys.exit(1)

        self.get_logger().info("Modelo cargado correctamente.")

        # 3. Iniciar el hilo de escucha
        self.stop_threads = False
        self.thread = threading.Thread(target=self.loop_escucha)
        self.thread.daemon = True
        self.thread.start()

    def loop_escucha(self):
        """Bucle principal de escucha (se ejecuta en paralelo)"""
        # Inicializamos el reconocedor
        rec = KaldiRecognizer(self.model, 16000)
        p = pyaudio.PyAudio()
        
        try:
            # Abrimos el micrófono. 
            # Nota: No usamos supresores de error para evitar bloqueos.
            stream = p.open(format=pyaudio.paInt16, 
                            channels=1, 
                            rate=16000, 
                            input=True, 
                            input_device_index=self.mic_index,
                            frames_per_buffer=4000)
            
            stream.start_stream()
            self.get_logger().info(f"Escuchando... Di '{self.trigger_word}' para activar.")

            while not self.stop_threads:
                # Leemos el buffer de audio
                data = stream.read(4000, exception_on_overflow=False)
                
                # Vosk procesa el audio
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    texto = result['text']
                    
                    if texto:
                        # Solo imprimimos si detectó texto real (evita spam de vacíos)
                        self.get_logger().info(f"He entendido: '{texto}'")
                        self.procesar_orden(texto)

        except Exception as e:
            self.get_logger().error(f"Error en el sistema de audio: {e}")
            self.get_logger().warn("Verifica que el micrófono está conectado y seleccionado en Configuración > Sonido")
        finally:
            if 'stream' in locals():
                stream.stop_stream()
                stream.close()
            p.terminate()

    def procesar_orden(self, texto):
        """Lógica de comandos"""
        texto = texto.lower()

        if self.trigger_word in texto or "lázaro" in texto:
            self.activar_robot()
        
        elif "adios" in texto or "adiós" in texto:
            self.get_logger().info("Comando recibido: Adiós")
            # El '&' es vital para que el robot siga escuchando mientras habla
            os.system('espeak -v es "Hasta luego compañero" &') 

    def activar_robot(self):
        self.get_logger().info('¡ACTIVADO! Esperando órdenes...')
        
        # 1. Feedback visual
        print("\n-------------------------")
        print("   ESTOY ESCUCHANDO      ")
        print("-------------------------\n")

        # 2. Sonido del Kobuki (Beep)
        sonido_msg = Sound()
        sonido_msg.value = 6 
        self.publisher_sound.publish(sonido_msg)

        # 3. Feedback de voz del PC
        os.system('espeak -v es "Dime, te escucho" &') 

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