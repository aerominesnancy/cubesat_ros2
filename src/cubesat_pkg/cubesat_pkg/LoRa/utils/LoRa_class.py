import struct
import serial
import time
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("\n[ERROR] RPi.GPIO module not found.","\nThe only functionalities available will be :",
          "\n - data encapsulation : LoRa.encapsulate() ", "\n - buffer management : Buffer()")


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
    id_to_type = {0x01: int(),
                  0x02: str()}
    type_to_id = {v: k for k, v in id_to_type.items()}
    logger = Raise_Errors_Logger()

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
    

    def send_radio(self, message: str):
        
        if not self.wait_aux():
            self.logger.error("Cannot send message because LoRa module is not ready.")
            return  # cannot send if AUX is not HIGH

        try:
            self.ser.write(self.encapsulate(message))
            #self.ser.flush()   # this line ensure data is sent but blocks the program and bypass timeout handling 
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


    def extract_message(self) -> str:
        return self.buffer.extract_message()


    def encapsulate(self, message) -> bytes:
        """ 
        [Marqueur de début (2 octets)]
        [Type de données (1 octet)]
        [Longueur (2 octets)]
        [Données (N octets)]
        [Marqueur de fin (2 octets)]
        """

        # Détermination du type de données et conversion en bytes
        if isinstance(message, int):
            data_type = self.type_to_id[int()]
            data_bytes = struct.pack('>i', message)  # entier 4 octets big-endian
        elif isinstance(message, str):
            data_type = self.type_to_id[str()]
            data_bytes = message.encode('utf-8')
        else:
            self.logger.error(f"Type de données {type(message)} non supporté.")
        
        length = len(data_bytes)

        return self.START_MARKER + struct.pack('>B', data_type) + struct.pack('>H', length) + data_bytes + self.END_MARKER

    def close(self):
        self.ser.close()
        GPIO.cleanup([self.M0, self.M1, self.AUX])
        self.logger.warn("LoRa serial port closed and GPIO cleaned up.")
        del self



class Buffer():
    def __init__(self, START_MARKER, END_MARKER, id_to_type, logger=Raise_Errors_Logger()):
        self.START_MARKER = START_MARKER
        self.END_MARKER = END_MARKER
        self.id_to_type = id_to_type

        self.logger = logger
        
        self.size = 0
        self.buffer = bytearray()
    
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
            return None
        
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
            data_bytes = full_message[5:5 + length]

            # validations
            if len(full_message) <= 2+1+2+2:
                self.clear()
                self.logger.error("Message trop court pour être valide. Buffer vidé.")
                return None
            if len(data_bytes) != length:
                self.clear()
                self.logger.error("Longueur des données incorrecte. Perte de paquets possible. Buffer vidé.")
                return None
            if data_type is None:
                self.clear()
                self.logger.error("Type de données inconnu. Buffer vidé.")
                return None

            return self.decode_message(data_type, data_bytes)
        
        else:
            return None  # No complete message found

    def decode_message(self, data_type, data_bytes):
        try:
            if data_type == int():
                return struct.unpack('>i', data_bytes)[0]
            
            elif data_type == str():
                return data_bytes.decode('utf-8')
            
        except Exception as e:
            self.logger.error(f"Erreur lors du décodage du message: {e}")
            return None

