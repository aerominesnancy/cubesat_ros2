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

        for baud in [4800, 9600, 19200, 38400, 57600, 115200]:
            self.ser = serial.Serial('/dev/ttyAMA1', baudrate=baud, timeout=1)

            line = None
            while not line:
                line = self.ser.readline()
                if not line:
                    self.get_logger().warn(f"No data received from GPS module.")
                self.get_logger().info(f'Received GPS data : {line}')
            self.ser.close()
            
        return    
        self.timer = self.create_timer(1.0, self.read_gps_data)

        self.get_logger().info('GPS node has been started.')

    def read_gps_data(self):
        try:
            line = self.ser.readline()
            if not line:
                self.get_logger().warn(f"No data received from GPS module.")
                return
            self.get_logger().info(f'Received GPS data : {line}')
            
            nmea = self.parse_nmea_sentence(line)
            self.get_logger().info(f'GPS data decoded : {nmea}')


        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')

    def parse_nmea_sentence(self, sentence):
        """
        Parse une phrase NMEA et retourne un dictionnaire
        avec type de message et champs.
        Ex: $GPGGA,123519,4807.038,N,01131.000,E,...
        """
        sentence = sentence.strip()
        if not sentence.startswith(b'$'):
            return None

        # Supprimer le checksum si présent
        if b'*' in sentence:
            data, checksum = sentence.split(b'*')
        else:
            data = sentence

        # Décoder en ASCII et split
        parts = data.decode('ascii').split(',')
        msg_type = parts[0][1:]  # enlever le $
        fields = parts[1:]

        return {'type': msg_type, 'fields': fields}
"""
# example of data received :
b'H\xa5\x95\xd5)u!\x00\x0c(\x12\x00\x00\x00\xac\x88\x04\x02\x00\x00\x00\x00\x00\x00\x18(\x84H\xe3\xa5\xa55\xac\xa0\xa5\xa5\x15\x08R\xa5\xa1\xa55\x15\x08B\xa5\xa5\xa5\xa5\xa5\xa5\x15\x08!\xa1\x94\n'
b'H\xa5\x95\xd5)i!\x00\x05-\x12\x00\x00\x00\x80\x88\x05B\x00\x00\x00\x00\x00\x00\x00\x80PH\xe3\xa5\xa55\xac\xa0\xa5\xa5\x15\x08R\xa5\xa1\xa55\x15\x08B\xa5\xa5\xa5\xa5\xa5\xa5\x15\x08!\xa1%\n'
b'H\xa5\x95\xd5)e!\x00\x05ae!!!\xf9\x88qe!!!!!!\r\x14\x92H\xe3\xa5\xa55\xac\xa0\xa5\xa5\x15\x08R\xa5\xa1\xa55\x15\x08B\xa5\xa5\xa5\xa5\xa5\xa5\x15\x08!\xa1%\n'

# explanations here (in fact it is not nmea...):
#https://circuitdigest.com/microcontroller-projects/interfacing-neo6m-gps-module-with-esp32

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