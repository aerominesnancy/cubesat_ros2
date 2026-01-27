# ===================================================================
# 
# Ce programme est dédié à l'antenne au sol du CubeSat. 
# Il permet d'envoyer et de recevoir des messages via le module LoRa.
# Il utilise la classe LoRa définie dans LoRa_class.py pour gérer la communication.
#                                                                                  
# ===================================================================

import time
from queue import Queue
from threading import Thread
from cubesat_pkg.LoRa.ground_antenna.ground_GUI import GroundGUI
from cubesat_pkg.LoRa.utils.LoRa_class import LoRa


class GUI_Logger():
        """ This logger is meant to write logs in the GUI log window."""
        def __init__(self, gui): self.gui = gui
        def info(self, msg): self.gui.add_hitory(f"[INFO] {msg}", self.gui.logs_box)
        def warn(self, msg): self.gui.add_hitory(f"[WARN] {msg}", self.gui.logs_box)
        def error(self, msg): self.gui.add_hitory(f"[ERROR] {msg}", self.gui.logs_box)
        def fatal(self, msg): self.gui.add_hitory(f"[FATAL] {msg}", self.gui.logs_box)


class LoRaGround():

    def __init__(self):
        # init LoRa
        self.GUI = GroundGUI()
        self.logger = GUI_Logger(self.gui)
        self.lora = LoRa(M0_pin=17, M1_pin=27, AUX_pin=22, logger=self.logger)

        # init message queue and callbacks
        self.message_queue = Queue()
        self.callbacks = {}

        # init threads
        self.running = True
        self.receiver_thread = Thread(target=self._receive_loop, daemon=True)
        self.receiver_thread.start()    
        self.processor_thread = Thread(target=self._process_messages, daemon=True)
        self.processor_thread.start() 

    def _receive_loop(self):
        """
        This function is called in a separate thread.
        It listens to the radio, extract the messages from buffer and puts the messages in a queue 'self.message_queue'.
        """
        while self.running:

            # listen read and exctract message from LoRa buffer
            self.lora.listen_radio()
            msg = self.lora.extract_message(remove_from_buffer=True)

            # if a message is available, put it in the queue and add it to the GUI history
            if msg is not None:
                self.message_queue.put(msg)
                self.GUI.add_hitory(msg, self.GUI.messages_box)

            time.sleep(0.1)

    def _process_messages(self):
        """
        This function is called in a separate thread.
        It processes the messages in the queue 'self.message_queue'.
        """
        while self.running:

            # if a message is available in queue, process it
            if not self.message_queue.empty():
                msg_type, message, checksum = self.message_queue.get()
                self.logger.info(f"Message traité (type : {msg_type}) : {message}")


                if msg_type in self.callbacks and self.callbacks[msg_type]:
                    self.callbacks[msg_type](message)
            time.sleep(0.1)

    def close(self):
        """
        This function is called to stop all threads and close the radio.
        """
        self.running = False
        self.receiver_thread.join()
        self.processor_thread.join()
        self.lora.close()


    def ask_for_picture(self, compression_factor=50, callback=None):
        self.lora.send_message(compression_factor, "ask_for_picture")
        self.callbacks["file_info"] = callback


    def handle_file_info(self, message, checksum):
        nb_of_paquets, _ = message
        self.logger.info(f"Nombre de paquets à transférer : {nb_of_paquets}")
        # Ici, vous pouvez lancer la réception des paquets de manière asynchrone





    def _wait_for_msg_type(self, message_type, timeout_s=5):
        start = time.time()
        self.logger.info(f"Waiting for message of type : {message_type}")

        while time.time()-start < timeout_s:
            self.lora.listen_radio()
            msg = self.lora.extract_message()
            
            if msg is not None:
                msg_type, message, checksum = msg
            
                if msg_type == message_type:
                    self.logger.info(f"Message de type {message_type} reçu, attente terminée.")
                    return message, checksum
                else:
                    self.logger.info(f"Message d'un autre type reçu (type : {msg_type}) : {message}")
                
        self.logger.warn(f"Timeout warning : Aucun message de type '{message_type}' reçu.")
        return None



    def _ask_for_file_transmission(self, file_path):
        self.lora.send_message(file_path, "ask_for_file_transmission")

        msg = self.wait_for_msg_type("file_info")
        if msg == None:
            self.lora.logger.error("Aucune réponse pour la demande de transmission. Demande annulée.")
            return
        nb_of_paquets, _ = msg

        self.lora.logger.info(f"Nombre de paquets a transférer pour le fichier demandé : {nb_of_paquets}")
        return nb_of_paquets


    def _ask_for_picture(self, compression_factor=50):
        self.lora.send_message(compression_factor, "ask_for_picture")

        msg = self.wait_for_msg_type("file_info")
        if msg == None:
            self.lora.logger.error("Aucune réponse pour la demande de transmission. Demande annulée.")
            return
        nb_of_paquets, _ = msg

        self.lora.logger.info(f"Nombre de paquets a transférer pour le fichier demandé : {nb_of_paquets}")
        return nb_of_paquets


    def _ask_for_paquet(self, paquet_index):
        self.lora.send_message(paquet_index, "ask_for_file_paquet")

        msg = self.wait_for_msg_type("file_paquet")
        if msg == None:
            self.logger.error("Aucune réponse pour la demande de paquet. Demande annulée.")
            return

        paquet_msg, _ = msg
        paquet_index_received, paquet_data = paquet_msg

        if paquet_index_received == paquet_index:
            self.logger.info(f"Reception du paquet {paquet_index}")
            return paquet_data
        else:
            self.logger.warn("Mauvais paquet reçu, attente de la prochaine reception...")


    def _picture_transfert(self, compression_factor=50):
        # demande de transmission et attente de l'indication du nombre de paquets
        nb_of_paquets = None
        while nb_of_paquets == None:
            nb_of_paquets = self.ask_for_picture(compression_factor)

        # demande de paquets et attente de reception
        file_data = b''
        for index in range(nb_of_paquets):
            paquet = None
            while paquet == None:
                paquet = self.ask_for_paquet(index)
            file_data += paquet

        # reconstruction du fichier
        with open("fichier_transmis.jpg", 'wb') as new_file:
            new_file.write(file_data)




if __name__ == '__main__':
    lora = LoRaGround()
