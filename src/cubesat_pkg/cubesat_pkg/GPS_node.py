import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from datetime import datetime, timezone # to convert UTC time to timestamp
import time
import serial
"""
The module uses NEO-6M chip. It currently send RMC, VTG GGA, GSA, GSV(several times), GLL messages (in this order)
and one binary message (certainly UBX) that is not decoded yet.

explanations here :
https://circuitdigest.com/microcontroller-projects/interfacing-neo6m-gps-module-with-esp32
https://fr.wikipedia.org/wiki/NMEA_0183

"""
# ===============================================================================
# all fields end with "\r\n" (CR-LF)



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
        self.buffer = b'' # create a buffer to work on data

        self.timer = self.create_timer(1.0, self.read_gps_data)

        self.get_logger().info('GPS node has been started.')
        

    def read_gps_data(self):
        """
        messages order : RMC, VTG, GGA, GSA, GSV(several times), GLL, ubx binary
        """

        try:
            data = None
            data = self.read_buffer()
                    
        except Exception as e:
            self.get_logger().error(f"Error reading GPS buffer: {e}")
        
        try:
            if data:
                nmea = self.parse_nmea_sentence(data.decode('utf-8'))
                if not nmea:
                    self.get_logger().warn(f"Invalid NMEA sentence. {data}")
                else:
                    self.get_logger().info(f'GPS data decoded !')
                    self.print_gps_logs(nmea)

        except Exception as e:
            self.get_logger().error(f"Error parsing NMEA sentence: {e}")


    def read_buffer(self):

        if self.ser.in_waiting == 0:
            return
        
        self.buffer += self.ser.read(self.ser.in_waiting)
        NMEA_starter = b'$GPRMC'
        ubx_starter = b'\xb5b\x01\x03\x10\x00'

        # Remove all data after the last ubx message
        self.buffer = self.buffer[self.buffer.rfind(ubx_starter):]
        
        # extract the latest NMEA messages
        if not NMEA_starter in self.buffer:
            self.buffer = b'' # clear buffer
            return

        data = self.buffer[self.buffer.rfind(NMEA_starter):]
        self.buffer = b'' # clear buffer

        return data


    def parse_nmea_sentence(self, data):
        nmea = {"RMC":[], "VTG":[], "GGA":[], "GSA":[], "GSV":[], "GLL":[]}

        messages = data.split("$GP")
        for message in messages:
            msg_id = message[:3]
            msg_data = message[4:].split('*')[0] # remove checksum and first coma

            if msg_id == "GSV":
                nmea[msg_id].append(msg_data.split(","))
            else:
                nmea[msg_id] = msg_data.split(",")

        return nmea
    

    def print_gps_logs(self, nmea):
        time = nmea["RMC"][0]
        date = nmea["RMC"][8]
        status = nmea["RMC"][1]
        latitude = nmea["RMC"][2] # attention au signe !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        longitude = nmea["RMC"][4]# attention au signe !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        self.get_logger().info(f"============= GPS data ============\n"
            f"utc time : {time[:2]}:{time[2:4]}:{time[4:6]}\t date : {date} \t timestamp : {self.extract_timestamp(nmea)}\n"
            f"status : {status}\n"
            f"latitude : {latitude}\n"
            f"longitude : {longitude}\n"
            )
    
    def extract_timestamp(self, nmea):
        time_str = nmea["RMC"][0]
        date_str = nmea["RMC"][8]

        if not time_str or not date_str:
            return None

        hh = int(time_str[0:2])
        mm = int(time_str[2:4])
        ss = float(time_str[4:])

        day = int(date_str[0:2])
        month = int(date_str[2:4])
        year = 2000 + int(date_str[4:6])

        dt = datetime(
            year, month, day,
            hh, mm, int(ss),
            int((ss % 1) * 1e6),
            tzinfo=timezone.utc
        )

        return dt.timestamp()

    



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