import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import Sound
import threading
import pyaudio
import json
import os
import sys
from vosk import Model, KaldiRecognizer
from contextlib import contextmanager
from ctypes import *

# --- BLOQUE PARA SILENCIAR ERRORES DE ALSA (Del Programa 1) ---
# Esto evita que la terminal se llene de basura de logs de audio
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    try:
        asound = cdll.LoadLibrary('libasound.so')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None)
    except:
        yield
# --------------------------------------------------------------

class NodoEscuchaKobuki(Node):
    def __init__(self):
        super().__init__('nodo_escucha_kobuki')
        
        # Configuración
        self.mic_index = 3  # Tu índice de micrófono confirmado
        self.trigger_word = "lazaro" # Palabra de activación (minúsculas)

        # 1. Publicador para el sonido del Kobuki
        self.publisher_sound = self.create_publisher(Sound, '/commands/sound', 10)
        
        # 2. Cargar Modelo Vosk
        self.get_logger().info("Cargando modelo de voz (esto tarda un poco)...")
        try:
            # Asegúrate de que la carpeta 'model' está donde ejecutas el comando
            if not os.path.exists("/home/diego/Documents/ROBOTICA_3/MOVILES/Proyecto/model"):
                self.get_logger().error("¡ERROR! No encuentro la carpeta 'model'.")
                raise FileNotFoundError("Carpeta model no encontrada")
            self.model = Model("/home/diego/Documents/ROBOTICA_3/MOVILES/Proyecto/model")
        except Exception as e:
            self.get_logger().error(f"Error cargando modelo: {e}")
            sys.exit(1)

        self.get_logger().info("Modelo cargado. Iniciando hilo de escucha...")

        # 3. Iniciar el hilo de escucha (para no bloquear ROS)
        self.stop_threads = False
        self.thread = threading.Thread(target=self.loop_escucha)
        self.thread.daemon = True # El hilo muere si el programa principal muere
        self.thread.start()

    def loop_escucha(self):
        """Este bucle corre en paralelo y maneja el micrófono"""
        rec = KaldiRecognizer(self.model, 16000)
        p = pyaudio.PyAudio()
        
        try:
            with noalsaerr():
                stream = p.open(format=pyaudio.paInt16, 
                                channels=2, 
                                rate=16000, 
                                input=True, 
                                input_device_index=self.mic_index,
                                frames_per_buffer=4000)
            
            stream.start_stream()
            self.get_logger().info(f"Escuchando... Di '{self.trigger_word}'")

            while not self.stop_threads:
                # Leemos datos del micro
                data = stream.read(4000, exception_on_overflow=False)
                
                if rec.AcceptWaveform(data):
                    # Se ha completado una frase
                    result = json.loads(rec.Result())
                    texto = result['text']
                    
                    if texto:
                        self.get_logger().info(f"Texto detectado: '{texto}'")
                        self.procesar_orden(texto)
                else:
                    # (Opcional) Resultados parciales mientras hablas
                    pass

        except Exception as e:
            self.get_logger().error(f"Error en el hilo de audio: {e}")
        finally:
            if 'stream' in locals():
                stream.stop_stream()
                stream.close()
            p.terminate()

    def procesar_orden(self, texto):
        """Lógica para reaccionar a lo que se escucha"""
        texto = texto.lower()

        # Chequeamos si la palabra clave "lazaro" (o con tilde) está en la frase
        if self.trigger_word in texto or "lázaro" in texto:
            self.activar_robot()
        elif "adios" in texto or "adiós" in texto:
            self.get_logger().info("Comando: Adiós")
            os.system('espeak -v es "Hasta luego compañero"')
        else:
            # Opcional: imprimir qué entendió si no fue un comando
            pass

    def activar_robot(self):
        self.get_logger().info('¡PALABRA MÁGICA DETECTADA! -> Interactuando')
        
        # 1. Feedback visual en terminal
        print("\n--- LAZARO ACTIVADO ---\n")

        # 2. El Robot Pita (Publicar en ROS)
        sonido_msg = Sound()
        sonido_msg.value = 6  # Sonido POWER ON
        self.publisher_sound.publish(sonido_msg)

        # 3. El PC Habla (Feedback auditivo)
        # El '&' al final hace que espeak no bloquee el hilo brevemente
        os.system('espeak -v es "Hola, estoy listo" &') 

    def cleanup(self):
        self.stop_threads = True
        self.thread.join()

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoEscuchaKobuki()
    
    try:
        # ROS gira aquí gestionando cualquier otra cosa necesaria
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Deteniendo nodo...")
    finally:
        nodo.cleanup()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()