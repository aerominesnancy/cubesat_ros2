# ===================================================================
# 
# Ce programme est dédié à l'antenne au sol du CubeSat. 
# Il permet d'envoyer et de recevoir des messages via le module LoRa.
# Il utilise la classe LoRa définie dans LoRa_class.py pour gérer la communication.
#                                                                                  
# ===================================================================

import time
import threading
import RPi.GPIO as GPIO
from LoRa_class import LoRa, Just_Print_Logger

# Initialize LoRa module with GPIO pins and timeouts
lora = LoRa(M0_pin=17, M1_pin=27, AUX_pin=22, AUX_timeout=5, serial_timeout=5, logger=Just_Print_Logger())


# Thread to continuously receive messages
def receive_loop():
    while True:
        lora.listen_radio()
        message = lora.extract_message()
        if message is not None:
            print(f"[{time.strftime('%H:%M:%S')}] 📥 Reçu : {message}", flush=True)

        time.sleep(0.1)

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