import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import Sound
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import threading
import pyaudio
import json
import os
import sys
import math
from vosk import Model, KaldiRecognizer

class NodoEscuchaKobuki(Node):
    def __init__(self):
        super().__init__('nodo_escucha_kobuki')
        
        # --- CONFIGURACIÓN ---
        self.mic_index = None 
        self.trigger_word = "lazaro"
        
        # Rutas de archivos
        self.directorio_actual = os.path.dirname(os.path.abspath(__file__))
        self.archivo_memoria = os.path.join(self.directorio_actual, "lugares.json")

        # Cargar memoria (o usar default si es la primera vez)
        self.diccionario_lugares = self.cargar_memoria()
        
        # Variables de posición
        self.current_x = None
        self.current_y = None
        # ---------------------

        # Publicadores y Suscriptores
        self.publisher_sound = self.create_publisher(Sound, '/commands/sound', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # Cargar Modelo Vosk
        self.get_logger().info("Cargando modelo de voz...")
        try:
            ruta_modelo = os.path.join(self.directorio_actual, "model")
            if not os.path.exists(ruta_modelo):
                raise FileNotFoundError(f"No se encuentra modelo en: {ruta_modelo}")
            self.model = Model(ruta_modelo)
        except Exception as e:
            self.get_logger().error(f"Error crítico cargando modelo: {e}")
            sys.exit(1)

        self.get_logger().info("Modelo cargado.")

        # Hilo de escucha
        self.stop_threads = False
        self.thread = threading.Thread(target=self.loop_escucha)
        self.thread.daemon = True
        self.thread.start()

    def amcl_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def cargar_memoria(self):
        """Carga los puntos desde un archivo JSON para no perderlos al reiniciar"""
        if os.path.exists(self.archivo_memoria):
            try:
                with open(self.archivo_memoria, 'r') as f:
                    datos = json.load(f)
                    self.get_logger().info(f"Memoria cargada: {len(datos)} lugares conocidos.")
                    return datos
            except Exception as e:
                self.get_logger().error(f"Error leyendo memoria: {e}")
        
        # Default si no hay archivo
        return {
            "cocina":  [1.65, 0.0],
            "puerta":  [4.07, 3.39],
            "salón":   [5.0, -2.0]
        }

    def guardar_memoria(self):
        """Guarda el diccionario actual en el disco duro"""
        try:
            with open(self.archivo_memoria, 'w') as f:
                json.dump(self.diccionario_lugares, f, indent=4)
            self.get_logger().info("Memoria actualizada y guardada en disco.")
        except Exception as e:
            self.get_logger().error(f"No se pudo guardar la memoria: {e}")

    def loop_escucha(self):
        rec = KaldiRecognizer(self.model, 16000)
        p = pyaudio.PyAudio()
        
        try:
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, 
                            input=True, input_device_index=self.mic_index, frames_per_buffer=4000)
            stream.start_stream()
            self.get_logger().info(f"Escuchando... Di '{self.trigger_word}'...")

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
            if 'stream' in locals(): stream.stop_stream(); stream.close()
            p.terminate()

    def procesar_orden(self, texto):
        texto = texto.lower()
        palabras = texto.split() # Convertimos la frase en lista de palabras

        # 1. COMANDO "AÑADE" (Nuevo Punto)
        if "añade" in palabras or "agrega" in palabras:
            self.registrar_nuevo_punto(palabras)

        # 2. COMANDO "LLÉVAME"
        elif "llevame" in texto or "llévame" in texto:
            destino_encontrado = None
            for lugar in self.diccionario_lugares:
                if lugar in texto:
                    destino_encontrado = lugar
                    break
            
            if destino_encontrado:
                coords = self.diccionario_lugares[destino_encontrado]
                self.iniciar_navegacion(destino_encontrado, coords)
            else:
                self.get_logger().warn("Lugar no reconocido.")
                os.system('espeak -v es "No conozco ese lugar" &')

        # 3. UBICACIÓN
        elif "donde estoy" in texto or "dónde estoy" in texto:
            self.responder_ubicacion()

        # 4. ACTIVACIÓN
        elif self.trigger_word in texto or "lázaro" in texto:
            self.activar_robot()
        
        # 5. DESPEDIDA
        elif "adios" in texto or "adiós" in texto:
            os.system('espeak -v es "Hasta luego" &') 

    def registrar_nuevo_punto(self, palabras):
        """Lógica para aprender un nuevo lugar"""
        if self.current_x is None:
            os.system('espeak -v es "No se donde estoy, no puedo guardar el punto" &')
            return

        try:
            # Buscamos la palabra clave
            if "añade" in palabras:
                idx = palabras.index("añade")
            else:
                idx = palabras.index("agrega")

            # Cogemos la SIGUIENTE palabra como nombre
            # Ejemplo: "Añade garaje aqui" -> idx de añade es 0, nombre es palabras[1] ("garaje")
            if idx + 1 < len(palabras):
                nombre_nuevo = palabras[idx + 1]
                
                # Evitamos guardar palabras basura como "el", "la", "un"
                if nombre_nuevo in ["el", "la", "un", "punto", "aquí"]:
                     os.system('espeak -v es "Dame un nombre más específico" &')
                     return

                # Guardamos
                self.diccionario_lugares[nombre_nuevo] = [self.current_x, self.current_y]
                self.guardar_memoria() # Persistencia en disco
                
                msg = f"Guardado punto {nombre_nuevo}"
                self.get_logger().info(msg)
                os.system(f'espeak -v es "{msg}" &')
            else:
                os.system('espeak -v es "Qué nombre le pongo?" &')

        except Exception as e:
            self.get_logger().error(f"Error registrando punto: {e}")

    def responder_ubicacion(self):
        if self.current_x is None:
            os.system('espeak -v es "No estoy localizado" &')
            return

        lugar_mas_cercano = None
        distancia_minima = float('inf')
        margen_error = 1.5 

        for nombre, coords in self.diccionario_lugares.items():
            dist = math.hypot(self.current_x - coords[0], self.current_y - coords[1])
            if dist < distancia_minima:
                distancia_minima = dist
                lugar_mas_cercano = nombre

        if lugar_mas_cercano and distancia_minima <= margen_error:
            os.system(f'espeak -v es "Estamos en {lugar_mas_cercano}" &')
        else:
            os.system(f'espeak -v es "Estamos en zona desconocida" &')

    def iniciar_navegacion(self, lugar, coordenadas):
        x, y = coordenadas
        self.get_logger().info(f"🚀 Enviando a {lugar} ({x}, {y})")
        os.system(f'espeak -v es "Yendo a {lugar}" &')

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0

        self.goal_publisher.publish(msg)

    def activar_robot(self):
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