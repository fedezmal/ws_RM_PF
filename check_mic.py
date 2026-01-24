import pyaudio

p = pyaudio.PyAudio()
print("------------------------------------------------------------------")
print(f"{'INDEX':<5} | {'RATE':<8} | {'IN CH':<5} |NAME")
print("------------------------------------------------------------------")

for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    # Solo mostramos dispositivos que tengan canales de entrada > 0
    if info['maxInputChannels'] > 0:
        print(f"{i:<5} | {int(info['defaultSampleRate']):<8} | {info['maxInputChannels']:<5} | {info['name']}")
    else:
         print(f"{i:<5} | {'---':<8} | 0     | {info['name']} (Solo salida)")

p.terminate()