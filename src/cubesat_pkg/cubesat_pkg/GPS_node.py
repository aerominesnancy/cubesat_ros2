import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import time
import serial

# ===============================================================================
# all fields end with "\r\n" (CR-LF)

# GPS precision ~15m
# DGPS precision ~10m
# Dead Reckoning Mode : estimate position using previous position and speed
GPGGA_FIELDS = [
    "talker_sentence",      # $GPGGA
    "utc_time",             # hhmmss.sss
    "latitude",             
    "latitude_direction",   # N/S
    "longitude",
    "longitude_direction",  # E/W
    "fix_quality",          # 0=invalid, 1=GPS, 2=DGPS, 6=Dead Recknoning Mode (not available in version < v2.3)
    "num_satellites",       # 1 to 12
    "hdop",                 # Horizontal Dilution of Precision
    "altitude",             # MSL Altitude (sea level altitud)
    "altitude_units",       # m
    "geoid_separation",     # Ellipsoid altitude = MSL Altitude + Geoid Separation
    "geoid_units",          # m
    "age_of_diff_corr",     # seconds (null when DGPS is not used)
    "diff_ref_station_id",  # id 
    "checksum"
]

GPGLL_FIELDS = [
    "talker_sentence",      # $GPGLL
    "latitude",
    "latitude_direction",   # N/S
    "longitude",
    "longitude_direction",  # E/W
    "utc_time",             # hhmmss.sss
    "status",               # A=valid, V=invalid
    "mode",                 # N = not available only to NMEA version 2.3 and later
    "checksum"
]

GPGSA_FIELDS = [
    "talker_sentence",      # $GPGSA
    "mode",                 # 2D/3D mode switching : M=manual, A=auto 
    "fix_type",             # 1=no fix, 2=2D  (< 4 satellites), 3=3D (> 3 satellites)
    "satellite_1",
    "satellite_2",
    "satellite_3",
    "satellite_4",
    "satellite_5",
    "satellite_6",
    "satellite_7",
    "satellite_8",
    "satellite_9",
    "satellite_10",
    "satellite_11",
    "satellite_12",
    "pdop",                 # Position Dilution of Precision
    "hdop",                 # Horizontal Dilution of Precision
    "vdop",                 # Vertical Dilution of Precision
    "checksum"
]

GPGSV_FIELDS = [
    "talker_sentence",      # $GPGSV
    "number_of_messages",   # 1 to 3
    "message_number",
    "satellites_in_view",  

    # Satellite block (repeat 1 to 4 times)
    "sat_id",               # 1 to 32
    "sat_elevation",        # 0 to 90 degrees
    "sat_azimuth",          # 0 to 359 degrees (relative to true north)
    "sat_snr",              # 0 to 99 dBHz (null when not tracking)

    "checksum"
]

GPMSS = [] # not available for this module

GPRMC_FIELDS = [
    "talker_sentence",      # $GPRMC
    "utc_time",             # hhmmss.ss
    "status",               # A=valid, V=invalid
    "latitude",
    "latitude_direction",   # N/S
    "longitude",
    "longitude_direction",  # E/W
    "speed_over_ground",    # knots
    "course_over_ground",   # degrees (relative to the true North)
    "date",                 # ddmmyy
    "magnetic_variation",   # E=east, W=west
    "east_west_indicator",  # E=east
    "mode",                 # N = not available only to NMEA version 2.3 and later
    "checksum"              
]

GPVTG_FIELDS = [
    "talker_sentence",      # $GPVTG
    "course_true",          # degrees 
    "reference",            # T = True = relative to the true North
    "course_magnetic",      # degrees
    "reference",            # M = Magnetic = relative to the magnetic North
    "speed_knots",          # horizontal velocity
    "units",                # N = Knots
    "speed_kmh",            # horizontal velovity
    "K",                    # K = km/h
    "mode",                 # N = not available only to NMEA version 2.3 and later
    "checksum"
]

GPZDA = [] # not available for this module

# ===============================================================================


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
        # remove nois before NMEA sentence
        start_index = sentence.find(b"$GP")
        if start_index == -1:
            return None
        parasite = sentence[:start_index]
        sentence = sentence[start_index:]

        if parasite:
            self.get_logger().warn(f"Parasite characters found before NMEA sentence : {parasite}")

        # remove checksum
        sentence, checksum = sentence.split(b'*')

        # Ddecode ascii and split data
        data = sentence.decode('ascii', errors='ignore').strip()
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