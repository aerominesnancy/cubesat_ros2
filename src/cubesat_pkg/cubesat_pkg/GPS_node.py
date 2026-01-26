import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
# ros2 interface show sensor_msgs/msg/NavSatFix

from datetime import datetime, timezone # to convert UTC time to timestamp
import time
import serial

"""
https://www.coordonnees-gps.fr/

The module uses NEO-6M chip. It currently send NMEA0183 messages : 
RMC, VTG GGA, GSA, GSV(several times), GLL messages (in this order)
and one binary message (certainly UBX) that is not decoded yet.

explanations here :
https://circuitdigest.com/microcontroller-projects/interfacing-neo6m-gps-module-with-esp32
https://fr.wikipedia.org/wiki/NMEA_0183

"""
# ===============================================================================
# all fields end with "\r\n" (CR-LF)

GPRMC_FIELDS = [
    "utc_time",             # hhmmss.ss
    "status",               # A=valid, V=invalid
    "latitude",             # ddmm.mmmmm
    "latitude_direction",   # N/S
    "longitude",            # dddmm.mmmmm
    "longitude_direction",  # E/W
    "speed_over_ground",    # knots
    "course_over_ground",   # degrees (relative to the true North)
    "date",                 # ddmmyy
    "magnetic_variation",   # E=east, W=west
    "east_west_indicator",  # E=east
    "mode"                  # N = not available only to NMEA version 2.3 and later             
]

GPVTG_FIELDS = [
    "course_true",          # degrees 
    "reference",            # T = True = relative to the true North
    "course_magnetic",      # degrees
    "reference",            # M = Magnetic = relative to the magnetic North
    "speed_knots",          # horizontal velocity
    "unit",                 # N = Knots
    "speed_kmh",            # horizontal velovity
    "unit",                 # K = km/h
    "mode"                  # N = not available only to NMEA version 2.3 and later
]

# GPS precision ~15m
# DGPS precision ~10m
# Dead Reckoning Mode : estimate position using previous position and speed
GPGGA_FIELDS = [
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
    "diff_ref_station_id"   # id 
]

GPGSA_FIELDS = [
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
    "vdop"                  # Vertical Dilution of Precision
]

GPGSV_FIELDS = [
    "number_of_messages",   # 1 to 3
    "message_number",
    "satellites_in_view",  

    # Satellite block (repeat 1 to 4 times)
    "sat_id",               # 1 to 32
    "sat_elevation",        # 0 to 90 degrees
    "sat_azimuth",          # 0 to 359 degrees (relative to true north)
    "sat_snr"               # 0 to 99 dBHz (null when not tracking)
]

GPGLL_FIELDS = [
    "latitude",
    "latitude_direction",   # N/S
    "longitude",
    "longitude_direction",  # E/W
    "utc_time",             # hhmmss.sss
    "status",               # A=valid, V=invalid
    "mode"                  # N = not available only to NMEA version 2.3 and later
]



# ===============================================================================


class GPS(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.is_valid = True

        # Récupération des paramètres
        callback_delay_second = self.declare_parameter('callback_delay_second', -1.0).value
        
        if callback_delay_second == -1:
            self.get_logger().error("Parameter 'callback_delay_second' must be set to a positive float."
                                    + f" Current value : {callback_delay_second}")
            self.get_logger().warn("GPS node is shutting down...")
            self.is_valid = False

        else:
            # uart2 are GPIO (for raspberry pi 4) 12 (TX) and 13 (RX)
            # ttyAMA* index can change depending on the number of serial port on the raspberry pi
            # GPS module use baud=38400 by default
            self.ser = serial.Serial('/dev/ttyAMA1', baudrate=38400, timeout=1)

            # variables
            self.buffer = b'' # create a buffer to work on data
            self.nmea = None

            #timer and publisher
            self.publisher = self.create_publisher(NavSatFix, '/gps/data', 1)
            self.timer = self.create_timer(callback_delay_second, self.send_gps_data)

            self.get_logger().info('GPS node has been started.')

    
    def send_gps_data(self):
        self.nmea = self.read_gps_data()
        if not self.nmea:
            return
        
        if self.nmea["RMC"][1] == "A":
            self.get_logger().info(f'GPS data decoded and valid.')
            self.print_gps_data_for_user()
            
            # convert position
            latitude, longitude, altitude = self.convert_to_decimal_degrees(
                self.nmea["RMC"][3].strip(), self.nmea["RMC"][2], # latitude  direction / latitude
                self.nmea["RMC"][5].strip(), self.nmea["RMC"][4], # longitude direction / longitude
                self.nmea["GGA"][8])                              # altitude
            
            # publish data
            self.publisher.publish(NavSatFix(
                status = NavSatStatus(status=NavSatStatus.STATUS_FIX),
                latitude = latitude,
                longitude = longitude,
                altitude = altitude))

        else:
            self.get_logger().warn(f'GPS data decoded but invalid : No satellites in view, try moving gps module.')
            self.publisher.publish(NavSatFix(
                status = NavSatStatus(status=NavSatStatus.STATUS_NO_FIX)))
        
        self.get_logger().info(f"GPS data published on topic : '/gps/data'")
              
        
    def read_gps_data(self):
        """
        messages order : RMC, VTG, GGA, GSA, GSV(several times), GLL, ubx binary
        """

        try:
            data = self.read_buffer()
            if not data:
                self.get_logger().warn(f"Not enough data in buffer. No NMEA message available.")
                return
            data = data.decode('utf-8') 

        except Exception as e:
            self.get_logger().error(f"Error reading GPS buffer: {e}")
            return
        
        try:
            nmea = self.parse_nmea_sentence(data)
            return nmea
        
        except Exception as e:
            self.get_logger().error(f"Error parsing NMEA sentence: {e}")
            return


    def read_buffer(self, NMEA_starter = b'$GPRMC', ubx_starter = b'\xb5b\x01\x03\x10\x00'):
        """Reads the serial buffer and returns the latest NMEA message."""

        # read gps buffer if available
        if self.ser.in_waiting == 0:
            return
        self.buffer += self.ser.read(self.ser.in_waiting)
        
        # find the last ender
        end_index = self.buffer.rfind(ubx_starter)
        if end_index == -1:
            # no ender found, wait until the next reading
            return
        
        # find the last starter before this ender
        start_index = self.buffer[:end_index].rfind(NMEA_starter)
        if start_index == -1:
            # no starter found, clear buffer before the ender
            self.buffer = self.buffer[end_index:]
            return

        # extract data and remove it from the buffer
        data = self.buffer[start_index:end_index]
        self.buffer = self.buffer[end_index:] # clear buffer

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
    

    def print_gps_data_for_user(self):
        nmea = self.nmea

        time = nmea["RMC"][0]
        date = nmea["RMC"][8]
        if time and date:
            timestamp = self.extract_timestamp(nmea)


        num_sat = 0
        for i in range(2,14):
            if nmea["GSA"][i]:
                num_sat += 1
        fix_quality = {0:'invalid', 1:'GPS', 2:'DGPS', 6:"estimated"}[int(nmea["GGA"][5])]
        fix_type = {1:'no fix', 2:'2D fix', 3:'3D fix'}[int(nmea["GSA"][1])]

        latitude = nmea["RMC"][2] 
        lat_dir = nmea["RMC"][3]
        longitude = nmea["RMC"][4]
        long_dir = nmea["RMC"][5]
        latitude, longitude = self.convert_to_degrees_minutes_seconds(latitude, longitude)
        h_precision = nmea["GSA"][15]
        h_precision = '?' if h_precision=='1.0' else h_precision

        altitude = nmea["GGA"][8] # école des Mines ~250m
        v_precision = nmea["GSA"][16]
        v_precision = '?' if v_precision=='1.0' else v_precision
        # Precisions are DOP (Dilution of Precision)
        # to obtain the precision in meters, multiply by the HDOP/VDOP by the precision of the GPS receiver
        
        speed = nmea["VTG"][6]
        course_angle = nmea["VTG"][0]
        course_angle = "?" if not course_angle else course_angle
        

        self.get_logger().info(f"============= GPS data ============\n"
            f"ATOMIC CLOCKS : \t utc: {time[:2]}:{time[2:4]}:{time[4:6]} \t date: {date} \t timestamp:{timestamp}\n"
            f"number of satellites : {num_sat}\t\t fix quality : {fix_quality}\t\t fix type : {fix_type}\n"
            f"latitude : {lat_dir}{latitude}\t\tlongitude : {long_dir}{longitude} \t\t (precision: {h_precision})\n"
            f"altitude : {altitude}MSL (precision: {v_precision})\n"
            f"velocity : {speed}km/h\t\t direction : {course_angle}° (compare to true North)"
            )
        
        
    def convert_to_degrees_minutes_seconds(self, latitude, longitude):
        return (f"{int(latitude[:2])}°{int(latitude[2:4])}'{round(float(latitude[2:]) % 1 * 60, 2)}\"" ,
                f"{int(longitude[:3])}°{int(longitude[3:5])}'{round(float(longitude[3:]) % 1 * 60,2)}\"")
    
    def convert_to_decimal_degrees(self, lat_dir, latitude, long_dir, longitude, altitude):
        # change sign (East/West and North/South)
        lat_sign = 1 if "E" in lat_dir else -1
        long_sign = 1 if "N" in long_dir else -1

        # Convert "degrees and minutes" to "decimal degrees"
        lat  = round(float(latitude[:2])  + float(latitude[2:])  / 60, 4) # 4 decimals ~ 10m precision
        long = round(float(longitude[:3]) + float(longitude[3:]) / 60, 4)

        # Convert altitude
        alt = float(altitude)

        return lat_sign*lat, long_sign*long, alt 


    def extract_timestamp(self, nmea):
        time_str = nmea["RMC"][0]
        date_str = nmea["RMC"][8]

        if not time_str or not date_str:
            return None

        hh = int(time_str[0:2])
        mm = int(time_str[2:4])
        ss = int(time_str[4:6])
        sss = int((float(time_str[4:]) % 1) * 1e6)

        day = int(date_str[0:2])
        month = int(date_str[2:4])
        year = 2000 + int(date_str[4:6])

        dt = datetime(
            year, month, day,
            hh, mm, ss, sss,
            tzinfo=timezone.utc
        )

        return round(dt.timestamp(),2)

    



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