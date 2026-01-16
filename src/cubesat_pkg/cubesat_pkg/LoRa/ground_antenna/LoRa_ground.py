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


lora.send_radio(None, "picture_ask")

while True:
        lora.listen_radio()
        msg_type, message = lora.extract_message()
        if message is not None and msg_type!="picture":
            print(f"[{time.strftime('%H:%M:%S')}] 📥 Reçu : {message}", flush=True)
        
        if msg_type == "picture":
            print("Image reçu")
            print(message)

        time.sleep(0.1)



"""
# Thread to continuously receive messages
def receive_loop():
    while True:
        lora.listen_radio()
        msg_type, message = lora.extract_message()
        if message is not None and msg_type!="picture":
            print(f"[{time.strftime('%H:%M:%S')}] 📥 Reçu : {message}", flush=True)
        
        if msg_type == "picture":
            print("Image reçu")
            print(message)

        time.sleep(0.1)

receiver_thread = threading.Thread(target=receive_loop, daemon=True)
receiver_thread.start()


# test sending messages
lora.send_radio("test string", "string")
time.sleep(1)
lora.send_radio(42, "int")
time.sleep(1)
lora.send_radio(time.time(), "timestamp_update")
time.sleep(1)
lora.send_radio(None, "picture_ask")

print("End of main thread, quit with Ctrl+C to stop receiving.")
while True:
    try:
        time.sleep(0.1)
    except KeyboardInterrupt:
        break

lora.close()
"""

