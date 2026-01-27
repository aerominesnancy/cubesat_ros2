# ===================================================================
# 
# Ce programme est dédié à l'antenne au sol du CubeSat. 
# Il permet d'envoyer et de recevoir des messages via le module LoRa.
# Il utilise la classe LoRa définie dans LoRa_class.py pour gérer la communication.
#                                                                                  
# ===================================================================

import time
from queue import Queue
from threading import Thread, Timer
from cubesat_pkg.LoRa.utils.LoRa_class import LoRa

##############################################################################################


class Just_Print_Logger():
    """ This logger just prints messages to the console without raising exceptions. 
    It works like the ros2 logging system (there is no ros2 environment on the ground antenna). """
    def info(self, msg):
        print(f"[INFO]  {msg}")

    def warn(self, msg):
        print(f"[WARN]  {msg}")

    def error(self, msg):
        print(f"[ERROR] {msg}")


##############################################################################################
##############################################################################################


class LoRaGround():

    def __init__(self, logger=None):
        # init LoRa
        self.logger = logger if logger else ExternalLogger(self) # use the UILogger if no logger is provided
        self.lora = LoRa(M0_pin=17, M1_pin=27, AUX_pin=22, logger=self.logger)

        # init variables
        self.gps_data = {"last_update":0.0, "status":-1, "latitude":0.0, "longitude":0.0, "altitude":0.0}

        self._reset_file_transfert()
        self.file_transfert_timeout = 5 # sec
        self.max_number_of_try = 5

        # init message queue and callbacks
        self.message_queue = Queue()
        self.callbacks = {"gps" : self.gps_callback}   
        self.external_observers = {"gps_update": [],            # list of functions to call in the UI for each event
                                   "new_message_received": [],
                                   "new_message_sent": [],
                                   "new_log": []}

        # init threads
        self.running = True
        self.receiver_thread = Thread(target=self._receive_loop, daemon=True)
        self.receiver_thread.start()    
        self.processor_thread = Thread(target=self._process_messages, daemon=True)
        self.processor_thread.start() 

        self.logger.info("LoRaGround initialized")

    ##################################### main functions #####################################
    def _receive_loop(self):
        """
        This function is called in a separate thread.
        It listens to the radio, extract the messages from buffer and puts the messages in a queue 'self.message_queue'.
        """
        while self.running:

            # listen read and exctract message from LoRa buffer
            self.lora.listen_radio()
            msg = self.lora.extract_message(remove_from_buffer=True)

            # if a message is available, put it in the queue
            if msg is not None:
                self.message_queue.put(msg)
                self._notify_observers("new_message_received", msg)

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

    def _send_message(self, message, msg_type):
        """
        Send a message to the cubesat.
        """
        self.lora.send_message(message, msg_type)
        self._notify_observers("new_message_sended", (msg_type,message, None))

    def close_radio(self):
        """
        This function is called to stop all threads and close the radio.
        """
        self.running = False
        self.receiver_thread.join()
        self.processor_thread.join()
        self.lora.close()

    ################################ file transfert functions ################################
    def _reset_file_transfert(self):
        """
        Sets the file transfert variables to their default values.
        """
        self.file_name = "name.txt"
        self.nb_of_packets = -1
        self.current_packet_index = -1
        self.packets_list = []

    def ask_for_picture(self, compression_factor=50):
        """
        Ask the cubesat to send a picture.
        """
        # stop current file transfert and initialise a new one
        self.stop_file_transfert()
        self.file_name = "picture.jpg"

        # send request via LoRa and set callback for the response 'file_info'
        self._send_message(compression_factor, "ask_for_picture")
        self.callbacks["file_info"] = self._handle_file_info

    def _handle_file_info(self, message):
        """Callback for the 'file_info' message type."""
        # initialise the variables for the file transmission
        self.nb_of_packets = message
        self.packets_list = [None] * self.nb_of_packets
        self.logger.info(f"Nombre de paquets à transférer : {self.nb_of_packets}")

        # remove callback and ask for the first packet
        del self.callbacks["file_info"]
        self._ask_for_file_packet(0)

    def _ask_for_file_packet(self, packet_index, number_of_try=0):
        """Ask for a specific packet and set a timeout."""
        # send LoRa message and set callback for the response 'file_packet'
        self._send_message(packet_index, "ask_for_file_packet")
        self.callbacks["file_packet"] = self._handle_file_packet

        # start a timeout timer
        self.timeout_timer = Timer(self.file_transfert_timeout, self._on_packet_timeout, args=[packet_index, number_of_try])
        self.timeout_timer.start()

    def _on_packet_timeout(self, packet_index, number_of_try):
        """Callback for the timeout timer. Start if no packet is received in time. Run _ask_for_file_packet again."""
        # remove callback and stop timer
        self.logger.warn(f"Timeout ! Packet n°{packet_index} not received.")
        self.callbacks["file_packet"] = None
        self.timeout_timer.cancel()

        # retry if max number of try not reached
        if number_of_try < self.max_number_of_try:
            self.logger.info(f"Retry number {number_of_try+1}/{self.max_number_of_try} for packet {packet_index}")
            self._ask_for_file_packet(packet_index, number_of_try+1)
        
        # stop file transfert if max number of try reached
        else:
            self.logger.error(f"Max number of try reached for packet {packet_index}. Aborting file transfert.")
            self.stop_file_transfert()

    def _handle_file_packet(self, message):
        """Callback for the file packet message. Save the packet and ask for the next one if not all packets are received."""
        # check if the packet is the one we are waiting for
        packet_index, packet_data = message
        if packet_index != self.current_packet_index:
            self.logger.warn(f"Received packet n°{packet_index} but waiting for n°{self.current_packet_index}. Ignoring.")
            return
        self.timeout_timer.cancel() # remove packet timeout if the packet is received

        # save the packet
        self.packets_list[packet_index] = packet_data

        # ask for the next packet if not all packets are received
        if None not in self.packets_list:
            self._end_file_transmission()
        else:
            next_packet = self.packets_list.index(None)
            self._ask_for_file_packet(next_packet)

    def _end_file_transmission(self):
        self.logger.info("Transfert de fichier terminé.")
        with open(self.file_name, 'wb') as f:
            for packet in self.packets_list:
                f.write(packet)
        self.logger.info(f"Fichier {self.file_name} enregistré.")

    def stop_file_transfert(self):
        if "file_info" in self.callbacks:
            del self.callbacks["file_info"]
        if "file_packet" in self.callbacks:
            del self.callbacks["file_packet"]
        if hasattr(self, 'timeout_timer'):
            self.timeout_timer.cancel()

        self._reset_file_transfert()

    ################################## Other LoRa callbacks ###################################
    def _handle_gps(self, message):
        self.gps_data["status"] = message[0]
        self.gps_data["latitude"] = message[1]
        self.gps_data["longitude"] = message[2]
        self.gps_data["altitude"] = message[3]
        self.gps_data["last_update"] = time.time()

        self.logger.info(f"GPS callback : {message}")
        self._notify_observers("gps_update", self.gps_data)


    ############################# Create external callback for UI #############################

    def add_observer(self, event_type, observer):
        """Ajoute un callback pour un type d'événement donné."""
        if event_type in self.observers:
            self.external_observers[event_type].append(observer)
        else:
            self.logger.error(f"Unknown event type : {event_type}.")

    def _notify_observers(self, event_type, data):
        """Notifie tous les callbacks enregistrés pour un type d'événement."""
        if event_type in self.observers:
            for observer in self.observers[event_type]:
                observer(data)



##############################################################################################
##############################################################################################

class ExternalLogger:
    """Logger interne qui utilise _notify_observers de LoRaGround."""

    def __init__(self, lora_instance:LoRaGround):
        self.lora = lora_instance  # Référence à l'instance de LoRaGround

    def info(self, msg):
        print(f"[INFO]  {msg}")  # Optionnel : affiche dans la console
        self.lora._notify_observers("new_log", f"[INFO]  {msg}")

    def warn(self, msg):
        print(f"[WARN]  {msg}")
        self.lora._notify_observers("new_log", f"[WARN]  {msg}")

    def error(self, msg):
        print(f"[ERROR] {msg}")
        self.lora._notify_observers("new_log", f"[ERROR] {msg}")

    def fatal(self, msg):
        print(f"[FATAL] {msg}")
        self.lora._notify_observers("new_log", f"[FATAL] {msg}")



if __name__ == '__main__':
    lora = LoRaGround(Just_Print_Logger())
    time.sleep(15)
    lora.close_radio()

