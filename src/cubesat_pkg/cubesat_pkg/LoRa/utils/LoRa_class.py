import struct
import serial
import time
import numpy as np
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("\n[ERROR] RPi.GPIO module not found.","\nLoRa class is not available. The only functionalities available will be :",
          "\n - data encapsulation : encapsulate() ", "\n - buffer management : Buffer()")


class Raise_Errors_Logger():
    """ This logger raises exceptions on warnings and errors.
    It is used for testing and debugging purposes (cf. __main__ section).
    """
    def info(self, msg):
        print(f"[INFO] {msg}")
    def warn(self, msg):
        raise UserWarning(f"[WARN] {msg}")
    def error(self, msg):
        raise Exception(f"[ERROR] {msg}")

class Just_Print_Logger():
    """ This logger just prints messages to the console without raising exceptions. 
    It works like the ros2 logging system (there is no ros2 environment on the ground antenna). """
    def info(self, msg):
        print(f"[INFO] {msg}")

    def warn(self, msg):
        print(f"[WARN] {msg}")

    def error(self, msg):
        print(f"[ERROR] {msg}")




class LoRa():
    START_MARKER = b'\xAA\xBB'  # Marqueur de début
    END_MARKER = b'\xCC\xDD'    # Marqueur de fin
    id_to_type = {0x01: "int",
                  0x02: "string",
                  0x03: "timestamp_update",
                
                  0x90: "picture",
                  0x91: "picture_start",
                  0x92: "picture_data",
                  0x93: "picture_end",
                  0x94: "picture_ask"}
    
    type_to_id = {v: k for k, v in id_to_type.items()}

    def __init__(self, M0_pin=-1 , M1_pin=-1, AUX_pin=-1, 
                 AUX_timeout=5.0, serial_timeout=5.0,
                 logger=Raise_Errors_Logger()):
        
        self.logger = logger

        self.M0 = M0_pin
        self.M1 = M1_pin
        self.AUX = AUX_pin

        self.AUX_timeout = AUX_timeout
        self.serial_timeout = serial_timeout
        

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
                             self.logger)


    def wait_aux(self):
        """Waits until the AUX pin goes HIGH, indicating that the LoRa module is ready."""

        start_time = time.time()

        while (GPIO.input(self.AUX) == GPIO.LOW) and (time.time() - start_time < self.AUX_timeout):
            time.sleep(0.1)
        
        if GPIO.input(self.AUX) == GPIO.LOW:
            self.logger.warn("Timeout waiting for LoRa module to be ready (AUX pin HIGH).")
            return False
        
        return True
    

    def send_radio(self, message, type):
        
        if not self.wait_aux():
            self.logger.error("Cannot send message because LoRa module is not ready.")
            return  # cannot send if AUX is not HIGH

        try:
            encapsulated = self.encapsulate(message, type)
            if encapsulated is None:
                self.logger.error("Message encapsulation failed. Message not sent.")
                return
            
            self.ser.write(encapsulated)

        except serial.SerialTimeoutException:
            self.logger.error("Error sending message: Serial timeout.")


        if not self.wait_aux():
            self.logger.error("Try sending message for too long, message may not have been sent.")
            return  

        self.logger.info(f"Message envoyé : {message}")


    def listen_radio(self):
        if self.ser.in_waiting > 0:
            bytes_msg = self.ser.read(self.ser.in_waiting)
            self.buffer.append(bytes_msg)
            self.logger.info(f"Received some data. Buffer size: {self.buffer.size} bytes.")
        else:
            self.logger.info(f"No data received.")
            pass


    def extract_message(self) -> tuple:
        return self.buffer.extract_message()


    def encapsulate(self, message, msg_type) -> bytes:
        return encapsulate(message, msg_type, self.type_to_id, self.START_MARKER, self.END_MARKER, self.logger)
    

    def close(self):
        self.ser.close()
        GPIO.cleanup([self.M0, self.M1, self.AUX])
        self.logger.warn("LoRa serial port closed and GPIO cleaned up.")
        del self



def encapsulate(message, msg_type:str,type_to_id, START_MARKER, END_MARKER, logger=Raise_Errors_Logger()) -> bytes:
    """ 
    [Marqueur de début (2 octets)]
    [Type de données (1 octet)]
    [Longueur (2 octets)]
    [Données (N octets)]
    [Marqueur de fin (2 octets)]
    """

    if msg_type not in type_to_id:
        logger.error(f"Type de données {msg_type} non supporté pour l'encapsulation.")
        return None

    # Détermination du type de données et conversion en bytes
    if msg_type == "int":
        if isinstance(message, int):
            data_type = type_to_id["int"]
            data_bytes = struct.pack('>i', message) 
        else:
            logger.error(f"Le message doit être de type int pour l'encapsulation de type 'int'.")
            return None
        
    elif msg_type == "string":
        if isinstance(message, str):
            data_type = type_to_id["string"]
            data_bytes = message.encode('utf-8')
        else:
            logger.error(f"Le message doit être de type str pour l'encapsulation de type 'string'.")
            return None
        
    elif msg_type == "timestamp_update":
        if isinstance(message, (int, float)):
            data_type = type_to_id["timestamp_update"]
            data_bytes = struct.pack('>I', int(message)) 
        else:
            logger.error(f"Le message doit être de type int ou float pour l'encapsulation de type 'timestamp_update'.")
            return None
    
    elif msg_type == "picture":
        if not isinstance(message, type(np.array(0))):
            logger.error(f"Le message doit être de type numpy.array pour l'encapsulation de type 'picture'.")
            return None
        
        nb_line, nb_column = message.shape

        # un premier message indique la taille de l'image
        start_message = (START_MARKER + struct.pack('>B', type_to_id["picture_start"]) + struct.pack('>H', 4) 
                        + struct.pack(">H", nb_line) 
                        + struct.pack(">H", nb_column) 
                        + END_MARKER )
        
        # on envoie l'image ligne par ligne
        data_message = bytearray()
        for i in range(nb_line):
            line_byte = bytearray()
            for j in range(nb_column):
                value = max(0,min(255, int(message[i][j])))
                line_byte += struct.pack(">B", value)

            data_message += (START_MARKER + struct.pack('>B', type_to_id["picture_data"]) + struct.pack('>H', nb_column+2) 
                            + struct.pack(">H", i)
                            + line_byte
                            + END_MARKER )
        
        end_message = (START_MARKER + struct.pack('>B', type_to_id["picture_end"]) + struct.pack('>H', 4) 
                        + struct.pack(">H", nb_line) 
                        + struct.pack(">H", nb_column) 
                        + END_MARKER )
        
        return start_message+start_message + data_message + end_message+end_message
    
    elif msg_type == "picture_ask":
        return START_MARKER + struct.pack('>B', type_to_id["picture_ask"]) + struct.pack('>H', 0) + END_MARKER

    length = len(data_bytes)

    return START_MARKER + struct.pack('>B', data_type) + struct.pack('>H', length) + data_bytes + END_MARKER



class Buffer():
    def __init__(self, START_MARKER, END_MARKER, id_to_type, logger=Raise_Errors_Logger()):
        self.START_MARKER = START_MARKER
        self.END_MARKER = END_MARKER
        self.id_to_type = id_to_type

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

    def extract_message(self) -> str:
        start_index = self.buffer.find(self.START_MARKER)
        end_index = self.buffer.find(self.END_MARKER, start_index + 2)

        # si on detecte des reliquats dans le buffer, on le vide
        if start_index == -1 and self.size > 0:
            self.clear()
            self.logger.warn("Reliquats détecté dans le buffer, buffer vidé.")
            return None,None
        
        # si on detecte des données avant le premier message, on les supprime
        elif start_index > 0 and end_index > start_index:
            self.buffer = self.buffer[start_index:]
            end_index -= start_index

            start_index = 0
            self.size = len(self.buffer)

            self.logger.warn("Message incomplet au début du buffer. Données précédentes supprimées.")
            # on ne retourne rien pour 

        # si on a trouvé un message complet au debut du buffer
        if start_index == 0 and end_index != -1:
            full_message = self.buffer[:end_index + 2]

            # on enlève le message complet du buffer
            self.buffer = self.buffer[end_index + 2:]
            self.size = len(self.buffer)

            data_type = self.id_to_type.get(full_message[2], None)
            length = struct.unpack('>H', full_message[3:5])[0]
            data_bytes = full_message[5:-2]

            # validations
            if len(full_message) < 2+1+2+2:
                self.clear()
                self.logger.error(f"Message trop court pour être valide ({full_message}). Buffer vidé.")
                return None,None
            if len(data_bytes) != length:
                self.clear()
                self.logger.error(f"Longueur des données incorrecte (réel:{len(data_bytes)} != indiqué:{length}). Perte de paquets possible. Buffer vidé.")
                return None,None
            if data_type is None:
                self.clear()
                self.logger.error("Type de données inconnu. Buffer vidé.")
                return None,None

            return data_type, self.decode_message(data_type, data_bytes)
        
        else:
            return None,None  # No complete message found

    def decode_message(self, data_type, data_bytes):
        try:
            if data_type == "int":
                return struct.unpack('>i', data_bytes)[0]
            
            elif data_type == "string":
                return data_bytes.decode('utf-8')
            
            elif data_type == "timestamp_update":
                return struct.unpack('>I', data_bytes)[0]
            
            elif "picture" in data_type:
                if data_type == "picture_ask":
                    return None

                elif data_type == "picture_start" and self.picture_mode == False:
                    number_of_lines, number_of_column = struct.unpack('>HH', data_bytes)
                    self.current_picture = np.zeros((number_of_lines, number_of_column))

                    self.picture_mode = True
                    self.logger.info(f"Début de la reception d'une image de taille {number_of_column}*{number_of_lines} pixels.")

                    return None

                elif data_type == "picture_end" :

                    self.picture_mode = False
                    self.logger.info("Fin de la reception de l'image.")

                    return self.current_picture

                elif data_type == "picture_data" and self.picture_mode == True:
                    line_number = struct.unpack(">H", data_bytes[0:2])[0]
                    line_data = struct.unpack(">" + "B"*len(data_bytes[2:]),data_bytes[2:])

                    self.current_picture[line_number] = np.array(line_data)

                    self.logger.info(f"Reception de la ligne {line_number} de l'image.")
                    return self.current_picture[line_number]
                
                return None

        except Exception as e:
            self.logger.error(f"Erreur lors du décodage du message: {e}")
            return None

