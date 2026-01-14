import time
import threading
import RPi.GPIO as GPIO
from LoRa_data_encapsulation import LoRa, Buffer

lora = LoRa(M0_pin=17, M1_pin=27, AUX_pin=4, AUX_timeout=5, serial_timeout=5)


# Thread to continuously receive messages
def receive_loop():
    while True:
        lora.listen_radio()
        message = lora.extract_message()
        if message is not None:
            print(f"[{time.strftime('%H:%M:%S')}] 📥 Reçu : {message}")

receiver_thread = threading.Thread(target=receive_loop, daemon=True)
receiver_thread.start()


try:
    while True:
        to_send = input("Message à envoyer : ")
        if to_send.lower() == "exit":
            break
        lora.send_radio(to_send)

except KeyboardInterrupt:
    print("\n🛑 Arrêt demandé par l'utilisateur.")


lora.close()