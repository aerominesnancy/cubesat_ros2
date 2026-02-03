import serial
import time

ser = serial.Serial('/dev/ttyAMA1', baudrate=9600, timeout=1)

# Envoyer une commande UBX pour lire la configuration
command = b'\xB5\x62\x06\x00\x00\x00\x1A\x2E'
ser.write(command)

# Lire la réponse
time.sleep(2)
response = ser.read(ser.in_waiting)
print(response)
