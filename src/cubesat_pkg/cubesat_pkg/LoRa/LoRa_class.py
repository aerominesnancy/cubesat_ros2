import struct
import serial
import time
import RPi.GPIO as GPIO
import rclpy

class LoRa():
    START_MARKER = b'\xAA\xBB'  # Marqueur de début
    END_MARKER = b'\xCC\xDD'    # Marqueur de fin
    type_to_id = {0x01: int(),
                  0x02: str()}
    id_to_type = {v: k for k, v in type_to_id.items()}

    def __init__(self, M0_pin: int, M1_pin: int, AUX_pin: int, 
                 AUX_timeout: float, serial_timeout: float,
                 logger=None):
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
                             self.type_to_id,
                             self.logger)


    def wait_aux(self):
        """Waits until the AUX pin goes HIGH, indicating that the LoRa module is ready."""

        start_time = time.time()

        while (GPIO.input(self.AUX) == GPIO.LOW) and (time.time() - start_time < self.AUX_timeout):
            time.sleep(0.1)
        
        if GPIO.input(self.AUX) == GPIO.LOW:
            self.get_logger().warn("Timeout waiting for LoRa module to be ready (AUX pin HIGH).")
            return False
        
        return True
    

    def send_radio(self, message: str):
        
        if not self.wait_aux():
            self.get_logger().error("Cannot send message because LoRa module is not ready.")
            return  # cannot send if AUX is not HIGH

        try:
            self.ser.write(self.encapsulate(message))
            #self.ser.flush()   # this line ensure data is sent but blocks the program and bypass timeout handling 
        except serial.SerialTimeoutException:
            self.get_logger().error("Error sending message: Serial timeout.")


        if not self.wait_aux():
            self.get_logger().error("Try sending message for too long, message may not have been sent.")
            return  

        self.get_logger().info(f"Message envoyé : {message}")


    def listen_radio(self):
        if self.ser.in_waiting > 0:
            bytes_msg = self.ser.read(self.ser.in_waiting)
            self.buffer.append(bytes_msg)
            self.get_logger().info(f"Received some data. Buffer size: {self.buffer.size} bytes.")
        else:
            #self.get_logger().info(f"No message received.")
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
            raise ValueError(f"Type de données {type(message)} non supporté.")
        
        length = len(data_bytes)

        return self.START_MARKER + struct.pack('>B', data_type) + struct.pack('>H', length) + data_bytes + self.END_MARKER




class Buffer():
    def __init__(self, START_MARKER, END_MARKER, type_id, logger=None):
        self.START_MARKER = START_MARKER
        self.END_MARKER = END_MARKER
        self.type_id = type_id

        self.logger = logger
        
        self.size = 0
        self.buffer = bytearray()

    def append(self, data: bytes):
        self.buffer.extend(data)
        self.size += len(data)

    def extract_message(self) -> str:
        start_index = self.buffer.find(self.START_MARKER)
        end_index = self.buffer.find(self.END_MARKER, start_index + 2)

        # si on detecte un message incomplet au début du buffer, on le nettoie
        if start_index != 0:
            if start_index == -1:
                self.buffer = bytearray()
                self.size = 0
                if self.logger:
                    self.logger.warning("Aucun marqueur de début trouvé, buffer vidé.")
            else:
                self.buffer = self.buffer[start_index:]
                end_index -= start_index
                start_index = 0
                if self.logger:
                    self.logger.warning("Message incomplet au début du buffer. Données précédentes supprimées.")
            return None

        # si on a trouvé un message complet au debut du buffer
        if start_index == 0 and end_index != -1:
            full_message = self.buffer[:end_index + 2]
            self.buffer = self.buffer[end_index + 2:]

            data_type = self.type_id.get(full_message[2], None)
            length = struct.unpack('>H', full_message[3:5])[0]
            data_bytes = full_message[5:5 + length]

            if len(data_bytes) <= 2+1+2+2:
                if self.logger:
                    self.logger.error("Message trop court pour être valide.")
                return None
            if len(data_bytes) != length:
                if self.logger:
                    self.logger.error("Longueur des données incorrecte. Perte de paquets possible.")
                return None
            if data_type is None:
                if self.logger:
                    self.logger.error("Type de données inconnu.")
                return None

            if isinstance(data_type, str):
                message = data_bytes.decode('utf-8')
                return message
            elif isinstance(data_type, int):
                message = struct.unpack('>i', data_bytes)[0]
                return message
        else:
            return None  # No complete message found


if __name__ == "__main__":

    print("\n--- Protocole de test LoRa_data_encapsulation.py ---")
    # Test 1 : Encapsulation correcte d'un message string
    try:
        msg = "Hello, CubeSat!"
        print(f"\nTest 1 - Message de base : {msg} ({type(msg)})")
        encapsulated = encapsulate(msg)
        print(f"Test 1 - Encapsulation obtenue : {encapsulated}")

        msg = 1234
        print(f"Test 1 - Message de base : {msg} ({type(msg)})")
        encapsulated = encapsulate(msg)
        print(f"Test 1 - Encapsulation obtenue : {encapsulated}")
        
        print("✅ Test 1 OK: Encapsulation réussie.")
    except Exception as e:
        print(f"❌ Test 1 ECHEC: {e}")

    # Test 2 : Encapsulation d'un type non supporté
    try:
        msg = 12.34
        print(f"\nTest 2 - Valeur de base : {msg} ({type(msg)})")
        encapsulate(msg)
        print("❌ Test 2 ECHEC: Exception non levée pour type non supporté.")
    except ValueError as e:
        print(f"✅ Test 2 OK: Exception levée pour type non supporté. Message : {e}")

    # Test 3 : Extraction correcte d'un message
    try:
        buf = Buffer()
        msg = "Hello, CubeSat!"
        encapsulated = encapsulate(msg)
        buf.append(encapsulated)
        print("\nTest 3 - Message encapsulé ajouté au buffer: ", msg)
        print(f"Test 3 - Buffer après append : {buf.buffer}")
        result = buf.extract_message()
        print(f"Test 3 - Message extrait : {result}")
        assert result == msg
        print("✅ Test 3 OK: Extraction correcte du message.")
    except Exception as e:
        print(f"❌ Test 3 ECHEC: {e}")

    # Test 4 : Message incomplet au début du buffer
    try:
        buf = Buffer()
        encapsulated = encapsulate("test_4_incomplete")
        buf.append(b'xxxx' + encapsulated)
        print(f"\nTest 4 - Buffer après append : {buf.buffer}")
        buf.extract_message()
        print("❌ Test 4 ECHEC: Warning non levé pour message incomplet.")
    except Warning as w:
        print(f"✅ Test 4.1 OK: Warning levé pour message incomplet. Message : {w}")
    except Exception as e:
        print(f"❌ Test 4 ECHEC: Mauvaise exception levée: {e}")
    try:
        result = buf.extract_message()
        print(f"Test 4 - Message extrait après nettoyage : {result}")
        assert result == "test_4_incomplete"
        print(f"✅ Test 4.2 OK: Bon nettoyage du buffer.")
    except Warning as w:
        print(f"❌ Test 4.2 OK: Le buffer n'a pas été nettoyé. Message : {w}")

    # Test 5 : Type inconnu dans le message
    try:
        bad_type = b'\xAA\xBB\xFF\x00\x05hello\xCC\xDD'  # type 0xFF inconnu
        print(f"\nTest 5 - Message encapsulé avec type inconnu : {bad_type}")
        buf = Buffer()
        buf.append(bad_type)
        buf.extract_message()
        print("❌ Test 5 ECHEC: Exception non levée pour type inconnu.")
    except ValueError as e:
        print(f"✅ Test 5 OK: Exception levée pour type inconnu. Message : {e}")

    # Test 6 : Longueur incorrecte dans le message
    try:
        # Longueur annoncée 10, mais seulement 5 octets de données
        bad_length = b'\xAA\xBB\x02\x00\x0Ahello\xCC\xDD'
        print(f"\nTest 6 - Message encapsulé avec mauvaise longueur : {bad_length}")
        buf = Buffer()
        buf.append(bad_length)
        buf.extract_message()
        print("❌ Test 6 ECHEC: Exception non levée pour longueur incorrecte.")
    except ValueError as e:
        print(f"✅ Test 6 OK: Exception levée pour longueur incorrecte. Message : {e}")

    # Test 7 : Extraction d'un message en deux temps (incomplet puis complet)
    try:
        msg = "Test 7 - message fragmenté"
        encapsulated = encapsulate(msg)
        # On découpe le message en deux parties
        part1 = encapsulated[:len(encapsulated)//2]
        part2 = encapsulated[len(encapsulated)//2:]
        buf = Buffer()
        print(f"\nTest 7 - Message de base : {msg}")
        print(f"Test 7 - Encapsulation complète : {encapsulated}")
        print(f"Test 7 - Partie 1 envoyée : {part1}")
        buf.append(part1)
        result = buf.extract_message()
        print(f"Test 7 - Résultat après partie 1 : {result}")
        if result is None:
            print("✅ Test 7.1 OK: Aucun message extrait, message incomplet.")
        else:
            print("❌ Test 7.1 ECHEC: Un message a été extrait alors qu'il est incomplet.")
        print(f"Test 7 - Partie 2 envoyée : {part2}")
        buf.append(part2)
        result = buf.extract_message()
        print(f"Test 7 - Résultat après partie 2 : {result}")
        if result == msg:
            print("✅ Test 7.2 OK: Message extrait correctement après réception complète.")
        else:
            print("❌ Test 7.2 ECHEC: Le message extrait n'est pas correct.")
    except Exception as e:
        print(f"❌ Test 7 ECHEC: {e}")

    # test 8 : Message trop court pour être valide
    try:
        buf = Buffer()
        msg = b'\xAA\xBB\x01\x00\x00\xCC\xDD'  # message trop court
        print(f"\nTest 8 - Message vide ajouté au buffer: {msg}")
        buf.append(msg)
        print(buf.extract_message())
        print("❌ Test 8 ECHEC: Exception non levée pour message vide.")
    except Exception as e:
        print(f"✅ Test 8 OK: Exception levée pour message trop court. Message : {e}")

    
    print("\n--- Fin du protocole de test ---\n\n")
