import struct
import serial
import time
import numpy as np
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("\n[ERROR] RPi.GPIO module not found.","\nLoRa class is not available. The only functionalities available will be :",
          "\n - data encapsulation : encapsulate() ", "\n - buffer management : Buffer()")
import binascii


class LoRa():
    START_MARKER = b'\xAA\xBB'  # Marqueur de début
    END_MARKER = b'\xCC\xDD'    # Marqueur de fin
    id_to_type = {0x00: "ACK",
                  0x01: "int",
                  0x02: "string",
                  0x03: "gps",
                
                  0x89: "ask_for_picture", # demande de la dernière image capturée avec un certain niveau de compression
                  0x90: "ask_for_file_transmission", # demande le transfert d'un fichier
                  0x91: "ask_for_file_packet",      # demande un certain packet
                  0x92: "file_packet",              # renvoie un packet
                  0x93: "file_info"                 # renvoie le nombre de packets liés au transfert
                  }
    
    type_to_id = {v: k for k, v in id_to_type.items()}

    packet_size = 240
    wrapper_size = 9

    def __init__(self, M0_pin=-1 , M1_pin=-1, AUX_pin=-1, 
                 AUX_timeout_s=5.0, serial_timeout_s=5.0,
                 ACK_timeout_s=5.0,
                 logger=None):
        
        self.logger = logger

        self.M0 = M0_pin
        self.M1 = M1_pin
        self.AUX = AUX_pin

        self.AUX_timeout = AUX_timeout_s
        self.serial_timeout = serial_timeout_s
        self.ACK_timeout = ACK_timeout_s

        # setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M0, GPIO.OUT)
        GPIO.setup(self.M1, GPIO.OUT)
        GPIO.setup(self.AUX, GPIO.IN)

        # Normal mode (transmission and reception)
        GPIO.output(self.M0, GPIO.LOW)
        GPIO.output(self.M1, GPIO.LOW)
        

        # setup serial connection
        self.ser = serial.Serial(port='/dev/serial0', baudrate=9600, timeout=self.serial_timeout)
            
        # create buffer for incoming messages
        self.buffer = Buffer(self.START_MARKER, self.END_MARKER,
                             self.id_to_type,
                             self.wrapper_size,
                             self.logger)
        
        self.current_picture_data = None


    def wait_aux(self):
        """Waits until the AUX pin goes HIGH, indicating that the LoRa module is ready."""

        start_time = time.time()

        while (GPIO.input(self.AUX) == GPIO.LOW) and (time.time() - start_time < self.AUX_timeout):
            time.sleep(0.001)
        
        if GPIO.input(self.AUX) == GPIO.LOW:
            self.logger.warn("[loRa_Class.py] Timeout waiting for LoRa module to be ready (AUX pin HIGH).")
            return False
        
        return True
    


    def send_bytes(self, data_bytes):
        if not isinstance(data_bytes, bytes):
            self.logger.warn("[loRa_Class.py] The message must be encapsulated before sending ! No message sended.")

        if not self.wait_aux():
            self.logger.error("[loRa_Class.py] Cannot send message because LoRa module is not ready.")
            return  # cannot send if AUX is not HIGH

        try:
            self.ser.write(data_bytes)

        except serial.SerialTimeoutException:
            self.logger.error("[loRa_Class.py] Error sending message: Serial timeout.")


        if not self.wait_aux():
            self.logger.error("[loRa_Class.py] Try sending message for too long, message may not have been sent.")
            return  

        self.logger.info(f"[loRa_Class.py] Message envoyé !")


    def send_message(self, message, message_type:str):
        encapsulation = self.encapsulate(message, message_type)
        self.logger.info(f"[loRa_Class.py] Sending {message_type} : {message}")

        if encapsulation is not None:
            checksum, bytes_message = encapsulation
            self.send_bytes(bytes_message)
            return checksum
            


    def listen_radio(self):
        if self.ser.in_waiting > 0:
            bytes_msg = self.ser.read(self.ser.in_waiting)
            self.buffer.append(bytes_msg)
            self.logger.info(f"[loRa_Class.py] Received some data. Buffer size: {self.buffer.size} bytes.")
        else:
            #self.logger.info(f"No data received.")
            pass


    def extract_message(self, remove_from_buffer = True) -> tuple:
        return self.buffer.extract_message(remove_from_buffer)


    def encapsulate(self, message, msg_type) -> bytes:
        encapsulated = encapsulate(message, msg_type, self.type_to_id, self.packet_size-self.wrapper_size, self.START_MARKER, self.END_MARKER, self.logger)
        if encapsulated is None:
            self.logger.error("[loRa_Class.py] Message encapsulation failed.")
            return None
        return encapsulated

    def close(self):
        self.ser.close()
        GPIO.cleanup([self.M0, self.M1, self.AUX])
        self.logger.warn("[loRa_Class.py] LoRa serial port closed and GPIO cleaned up.")
        del self


# ==============================================================================================================================
# ==============================================================================================================================




def calculate_checksum(data_bytes):
    # utilisation du checksum CRC-16
    return struct.pack(">H", binascii.crc_hqx(data_bytes, 0xffff))

def encapsulate(message, msg_type:str,type_to_id, max_data_size, START_MARKER, END_MARKER, logger=None) -> bytes:
    """ 
    [Marqueur de début (2 octets)]
    [Checksum (2 octets)]
    [Type de données (1 octet)]
    [Longueur (2 octets)]
    [Données (N octets)]
    [Marqueur de fin (2 octets)]
    """

    if msg_type not in type_to_id:
        logger.error(f"[loRa_Class.py] Type de données {msg_type} non supporté pour l'encapsulation.")
        return None

    if msg_type =="gps":
        if isinstance(message, tuple) and len(message)==4 and isinstance(message[0], int) and isinstance(message[1], float) and isinstance(message[2], float) and isinstance(message[3], float):
            data_bytes = struct.pack('>ifff', *message)
        else:
            logger.error(f"[loRa_Class.py] Le message doit être un tuple de 4 éléments (int, float, float, float) pour l'encapsulation de type 'gps'. Message actuel : (type : {type(message)}) {message}")

    # envoie d'un ACK
    elif msg_type == "ACK":
        if isinstance(message, (bytes, bytearray)) and len(message)==2:
            data_bytes = message
        else:
            logger.error(f"[loRa_Class.py] Le message contenu dans un ACK doit être un checksum ('>H' : bytearray de longueur 2). Message actuel : (type : {type(message)}) {message}")
            return None

    # envoie d'un int 
    elif msg_type == "int":
        if isinstance(message, int):
            data_bytes = struct.pack('>i', message) 
        else:
            logger.error(f"[loRa_Class.py] Le message doit être de type 'int' pour l'encapsulation de type 'int'. Message actuel : (type : {type(message)}) {message}")
            return None
    
    # envoie d'un string 
    elif msg_type == "string":
        if isinstance(message, str):
            data_bytes = message.encode('utf-8')
        else:
            logger.error(f"[loRa_Class.py] Le message doit être de type 'str' pour l'encapsulation de type 'string'. Message actuel : (type : {type(message)}) {message}")
            return None
    
    # envoi d'un fichier
    elif msg_type == "ask_for_file_transmission":
        if not isinstance(message, str):
            logger.error(f"[loRa_Class.py] Le path du fichier doit être de type 'str' pour l'encapsulation de type 'ask_for_file_transmission'. Message actuel : (type : {type(message)}) {message}")
            return None
        else:
            data_bytes = message.encode("utf-8")
    
    elif msg_type == "ask_for_file_packet":
        if isinstance(message, int):
            data_bytes = struct.pack('>H', message) 
        else:
            logger.error(f"[loRa_Class.py] Le message doit être de type 'int' pour l'encapsulation de type 'ask_for_file_packet'. Message actuel : (type : {type(message)}) {message}")
            return None
        
    elif msg_type == "file_packet":
        if isinstance(message, tuple) and len(message)==2 and isinstance(message[0], int) and isinstance(message[1], bytes):
            packet_index, packet_data = message
            data_bytes = struct.pack(">H", packet_index) + packet_data
        else:
            logger.error(f"[loRa_Class.py] Le message doit être un tuple (int, bytes) pour l'encapsulation de type 'file_packet'. Message actuel : (type : {type(message)}) {message}")
            return None

    elif msg_type == "file_info":
        if isinstance(message, int):
            data_bytes = struct.pack('>H', message) 
        else:
            logger.error(f"[loRa_Class.py] Le message doit être de type 'int' pour l'encapsulation de type 'file_info'. Message actuel : (type : {type(message)}) {message}")
            return None
    
    # envoie d'une photo
    elif msg_type == "ask_for_picture":
        if isinstance(message, int) and message <= 100 and message >= 0:
            data_bytes = struct.pack('>B', message)
        else:
            logger.error(f"[loRa_Class.py] Le message (compression_factor) doit être de type 'int' et doit être compris entre 0 et 100 pour l'encapsulation de type 'ask_for_pictures' (50 par defaut)")


    # mise en forme de l'envoie
    data_type = type_to_id[msg_type]
    length = len(data_bytes)
    if length > max_data_size:
        logger.warn(f"[loRa_Class.py] Le message est trop long pour être envoyé en une seule fois ({len(data_bytes)} > {max_data_size} bytes)! Risque de perte de packets...")
        
    checksum = calculate_checksum(data_bytes)
    encapsulated_msg = START_MARKER + calculate_checksum(data_bytes) + struct.pack('>B', data_type) + struct.pack('>H', length) + data_bytes + END_MARKER

    return checksum , encapsulated_msg


# ==============================================================================================================================
# ==============================================================================================================================


class Buffer():
    def __init__(self, START_MARKER, END_MARKER, id_to_type, wrapper_size, logger=None):
        self.START_MARKER = START_MARKER
        self.END_MARKER = END_MARKER
        self.id_to_type = id_to_type
        self.wrapper_size = wrapper_size

        self.logger = logger
        
        self.size = 0
        self.buffer = bytearray()

        self.picture_mode = False
        self.current_picture = None
    
    def clear(self):
        self.buffer = bytearray()
        self.size = 0

    def append(self, data: bytes):
        self.buffer.extend(data)
        self.size += len(data)

    def extract_message(self, remove_from_buffer = True) -> str:
        start_index = self.buffer.find(self.START_MARKER)
        end_index = self.buffer.find(self.END_MARKER, start_index + 2)

        # si on detecte des reliquats dans le buffer, on le vide
        if start_index == -1 and self.size > 2:
            self.logger.info(f"[loRa_Class.py] Contenu du buffer : {self.buffer}")
            self.logger.warn("[loRa_Class.py] Reliquats détecté dans le buffer, buffer vidé.")
            self.clear()
            return None
        
        # si on detecte des données avant le premier message, on les supprime
        elif start_index > 0 and end_index > start_index:
            self.buffer = self.buffer[start_index:]
            end_index -= start_index

            start_index = 0
            self.size = len(self.buffer)

            self.logger.warn("[loRa_Class.py] Message incomplet au début du buffer. Données précédentes supprimées.")
            # on ne retourne rien pour 

        # si on a trouvé un message complet au debut du buffer
        if start_index == 0 and end_index != -1:

            # on enlève le message complet du buffer
            full_message = self.buffer[:end_index + 2]
            if remove_from_buffer:
                self.buffer = self.buffer[end_index + 2:]
                self.size = len(self.buffer)

            # 1ere verification
            if len(full_message) < self.wrapper_size:
                self.logger.error(f"[loRa_Class.py] Message trop court pour être valide ({full_message}). Données supprimées.")
                return None

            # decomposition du message
            checksum = full_message[2:4]
            data_type = self.id_to_type.get(full_message[4], None)
            length = struct.unpack('>H', full_message[5:7])[0]
            data_bytes = full_message[7:-2]

            # validations intégritée du message
            if checksum != calculate_checksum(data_bytes):
                self.logger.error(f"[loRa_Class.py] Checksum différent détecté (reçu : {checksum}, calculé : {calculate_checksum(data_bytes)}). Le message est invalide. Données supprimées.")
                return None
            if len(data_bytes) != length:
                self.logger.error(f"[loRa_Class.py] Longueur des données incorrecte (réel:{len(data_bytes)} != indiqué:{length}). Perte de packets possible. Données supprimées.")
                return None
            if data_type is None:
                self.logger.error("[loRa_Class.py] Type de données inconnu. Données supprimées.")
                return None


            decoded = self.decode_message(data_type, data_bytes)
            if decoded is not None:
                return data_type, decoded, checksum
            else:
                self.logger.warn("[loRa_Class.py] Le message n'a pas pu être décodé.")
        
        else:
            return None  # No complete message found

    def decode_message(self, data_type, data_bytes):
        try:
            if data_type == "gps":
                return struct.unpack(">ifff", data_bytes)  # (status, latitude, longitude, altitude)
            
            elif data_type == "ACK":
                return data_bytes
            
            elif data_type == "int":
                return struct.unpack('>i', data_bytes)[0]
            
            elif data_type == "string":
                return data_bytes.decode('utf-8')
            
            elif "file" in data_type:
                if data_type == "ask_for_file_transmission":
                    file_path = data_bytes.decode('utf-8')
                    return file_path
                
                elif data_type == "ask_for_file_packet":
                    packet_index = struct.unpack(">H", data_bytes)[0]
                    return packet_index

                elif data_type == "file_info":
                    nb_of_packets = struct.unpack(">H", data_bytes)[0]
                    return nb_of_packets

                elif data_type == "file_packet":
                    packet_index = struct.unpack(">H", data_bytes[0:2])[0]
                    packet_data = data_bytes[2:]
                    return (packet_index, packet_data)
                
            elif data_type == "ask_for_picture":
                compression_factor = struct.unpack(">B", data_bytes)[0]
                return compression_factor

        except Exception as e:
            self.logger.error(f"[loRa_Class.py] Erreur lors du décodage du message: {e}")
            return None

