import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import time
import serial

class GPS(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.is_valid = True

        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # uart2 are GPIO (for raspberry pi 4) 12 (TX) and 13 (RX)
        # ttyAMA* index can change depending on the number of serial port on the raspberry pi

        # GPS module use baud=38400 by default
        self.ser = serial.Serial('/dev/ttyAMA1', baudrate=38400, timeout=1)

        self.timer = self.create_timer(1.0, self.read_gps_data)

        self.get_logger().info('GPS node has been started.')


    def read_gps_data(self):
        try:
            line = self.ser.readline()
            if not line:
                self.get_logger().warn(f"No data received from GPS module.")
                return
            
            self.get_logger().warn(f"received : {line}")
            
            nmea = self.parse_nmea_sentence(line)
            if not nmea:
                self.get_logger().warn(f"Invalid NMEA sentence. {line}")
            else:
                self.get_logger().info(f'GPS data decoded : {nmea}')


        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')


    def parse_nmea_sentence(self, sentence):
        """
        Parse une phrase NMEA et retourne un dictionnaire
        avec type de message et champs.
        """
        sentence = sentence.decode('ascii', errors="ignore").strip()

        # on recherche le début du message si il y a des caractères parasites avant
        start_index = sentence.find("$GP")
        parasite = sentence[:start_index]
        data = sentence[start_index:]

        if parasite:
            self.get_logger().warn(f"Parasite characters found in NMEA sentence : {sentence}")

        # Supprimer le checksum si présent
        data, checksum = data.split('*') if '*' in data else data

        # Décoder en ASCII et split
        parts = data.split(',')
        msg_type = parts[0][1:]  # enlever le $
        fields = parts[1:]

        return {'type': msg_type, 'fields': fields}
    

"""
# explanations here (nmea with baud=38400):
#https://circuitdigest.com/microcontroller-projects/interfacing-neo6m-gps-module-with-esp32
#https://fr.wikipedia.org/wiki/NMEA_0183
"""


def main(args=None):
    rclpy.init(args=args)

    gps_node = GPS()

    # let the node "alive" until interrupted
    try :
        if gps_node.is_valid:
            rclpy.spin(gps_node)

    except KeyboardInterrupt:
        gps_node.get_logger().warn('GPS node interrupted and is shutting down...')

    finally:
        if rclpy.ok():  # if the node is still running
            time.sleep(1)  # wait for logs to be sent
            rclpy.shutdown()