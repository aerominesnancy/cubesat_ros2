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
logger = Just_Print_Logger()
lora = LoRa(M0_pin=17, M1_pin=27, AUX_pin=22, logger=logger)


# Thread to continuously receive messages
def receive_loop():
    while True:
        lora.listen_radio()
        msg = lora.extract_message(remove_from_buffer = False)
        if msg is not None:
            msg_type, message, _ = msg
            logger.info(f"Message received (type : {msg_type}) : {message}")

        time.sleep(0.1)

receiver_thread = threading.Thread(target=receive_loop, daemon=True)
receiver_thread.start()



def wait_for_msg_type(message_type, timeout_s=5):
    start = time.time()
    
    while time.time()-start < timeout_s:
        lora.listen_radio()
        msg = lora.extract_message()
        
        if msg is not None:
            msg_type, message, checksum = msg
        
            if msg_type == message_type:
                logger.info(f"Message de type {message_type} reçu, attente terminée.")
                return message, checksum
            
    logger.warn(f"Timeout warning : Aucun message de type '{message_type}' reçu.")
    return None



def ask_for_file_transmission(file_path):
    lora.send_message(file_path, "ask_for_file_transmission")

    msg = wait_for_msg_type("file_info")
    if msg == None:
        lora.logger.error("Aucune réponse pour la demande de transmission. Demande annulée.")
        return
    nb_of_paquets, _ = msg

    lora.logger.info(f"Nombre de paquets a transférer pour le fichier demandé : {nb_of_paquets}")
    return nb_of_paquets


def ask_for_paquet(paquet_index):
    lora.send_message(paquet_index, "ask_for_file_paquet")

    msg = wait_for_msg_type("file_paquet")
    if msg == None:
        logger.error("Aucune réponse pour la demande de paquet. Demande annulée.")
        return

    paquet_index_received, paquet_data = msg

    if paquet_index_received == paquet_index:
        logger.info(f"Reception du paquet {paquet_index}")
        return paquet_data
    else:
        logger.warn("Mauvais paquet reçu, attente de la prochaine reception...")



# demande de transmission et attente de l'indication du nombre de paquets
nb_of_paquets = None
while nb_of_paquets == None:
    nb_of_paquets = ask_for_file_transmission("/home/cubesat/ros2_ws/pictures/last_picture.jpg")

# demande de paquets et attente de reception
file_data = b''
for index in range(nb_of_paquets):
    paquet = None
    while paquet == None:
        paquet = ask_for_paquet(index)
    file_data += paquet

# reconstruction du fichier
with open("fichier_transmis.jpg", 'wb') as new_file:
    new_file.write(file_data)


lora.close()


"""
    def send_and_wait_ACK(self, checksum, data_bytes, number_of_try=10):
        ACK_received = False

        checksum, data_bytes = data_bytes
        self.send_bytes(data_bytes)
        ACK_received = self.wait_ACK(checksum)
        
        for i in range(2, number_of_try+1):
            if ACK_received:
                break
            self.logger.warn(f"No ACK received. Message sended again (try {i+2}/{number_of_try}). Please wait...")
            checksum, data_bytes = data_bytes
            self.send_bytes(data_bytes)
            ACK_received = self.wait_ACK(checksum)
            

"""

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
lora.send_bytes("test string", "string")
time.sleep(1)
lora.send_bytes(42, "int")
time.sleep(1)
lora.send_bytes(time.time(), "timestamp_update")
time.sleep(1)
lora.send_bytes(None, "picture_ask")

print("End of main thread, quit with Ctrl+C to stop receiving.")
while True:
    try:
        time.sleep(0.1)
    except KeyboardInterrupt:
        break

lora.close()
"""

