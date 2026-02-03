import serial

ser = serial.Serial('/dev/ttyAMA1', baudrate=9600, timeout=1)

# Envoyer une commande UBX pour lire la configuration
command = b'\xB5\x62\x06\x00\x00\x00\x1A\x2E'
ser.write(command)

# Lire la réponse
response = ser.read(100)
print(response)
