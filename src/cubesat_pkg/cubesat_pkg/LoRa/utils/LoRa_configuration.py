import serial
import RPi.GPIO as GPIO
import time

"""
Ce programme permet de lire la configuration d'un module LoRa 33S via UART.
La fonction read_configuration_33S() affiche toutes les valeurs de 
configuration du module disponible dans la datasheet.

Il est important de vérifier que la configuration est la même pour les 2 modules LoRa utilisés (émetteur et récepteur).
(Attention la methode de lecture de la configuration difere selon le modèle de module LoRa utilisé : 33s != 37s)
"""

# ========================= Datasheet values =========================

UART_BAUD = {
    0b000: "1200 bps",
    0b001: "2400 bps",
    0b010: "4800 bps",
    0b011: "9600 bps (default)",
    0b100: "19200 bps",
    0b101: "38400 bps",
    0b110: "57600 bps",
    0b111: "115200 bps",
}

PARITY = {
    0b00: "8N1 (default)",
    0b01: "8O1",
    0b10: "8E1",
    0b11: "8N1 (equiv 00)",
}

AIR_RATE = {
    0b000: "0.3 kbps",
    0b001: "1.2 kbps",
    0b010: "2.4 kbps (default)",
    0b011: "4.8 kbps",
    0b100: "9.6 kbps",
    0b101: "19.2 kbps",
    0b110: "38.4 kbps",
    0b111: "62.5 kbps",
}

TRANSMITTING_POWER = {
    0b00: "33dBm (default)",
    0b01: "30dBm",
    0b10: "27dBm",
    0b11: "241dBm",
}

SUB_PACKET_SETTING = {
    0b00: "240 bytes (default)",
    0b01: "128 bytes",
    0b10: "64 bytes",
    0b11: "32 bytes",
}

RSSI_AMBIENT_NOISE = {
    0b0: "Disable (default)",
    0b1: "Enable",
}

ENABLE_RSSI = {
    0b0: "Disable (default)",
    0b1: "Enable",
}

TRANSMISSION_MODE = {
    0b0: "Transparent transmission mode (default)",
    0b1: "Fixed point transmission mode",
}

ENABLE_REPEATER = {
    0b0: "Disable repeater function",
    0b1: "Enable repeater function",
}

LBT_ENABLE = {
    0b0: "Disable (default)",
    0b1: "Enable",
}

WOR_TRANSCEIVER_CONTROL = {
    0b0: "WOR receiver (default)",
    0b1: "WOR transmitter",
}

WOR_CYCLE = {
    0b000: "500ms",
    0b001: "1000ms",
    0b010: "1500ms",
    0b011: "2000ms (default)",
    0b100: "2500ms",
    0b101: "3000ms",
    0b110: "3500ms",
    0b111: "4000ms",
}


# ============================ Functions ============================

class lora33S_config():
    def __init__(self, pins:dict):
        self.pins = pins

        # init GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pins["M0"], GPIO.OUT)
        GPIO.setup(self.pins["M1"], GPIO.OUT)
        GPIO.setup(self.pins["AUX"], GPIO.IN)

        # put LoRa in configuration mode
        self.open_config_mode()

        # open serial connection
        self.serial = serial.Serial(port='/dev/serial0',  # Raspberry Pi main UART
                        baudrate=9600,
                        timeout=1)
        
        # wait module to be ready
        self.wait_aux(self.pins)

    def open_config_mode(self):
        """Put LoRa in configuration mode"""
        GPIO.output(self.pins["M0"], GPIO.LOW)
        GPIO.output(self.pins["M1"], GPIO.HIGH)

    def wait_aux(self):
        """Attend que le module soit prêt (AUX HIGH)"""
        i = 0
        print("Waiting AUX...")
        while GPIO.input(self.pins["AUX"]) == GPIO.LOW:
            print("busy",i)
            i += 1
            time.sleep(0.01)
        print("AUX ok !")

    def ask_for_configuration(self):
        # create config message
        start = 0x00
        end = 0x09
        trame = bytes([0xC1, start, end])
        
        # send message
        self.serial.write(trame)
        self.serial.flush()
        
        # wait module to be ready
        self.wait_aux(self.pins)

    def read_configuration(self, pins):
        # send message to module and wait for response
        self.ask_for_configuration()
        time.sleep(0.1)
        
        if self.serial.in_waiting > 0:
            response = self.serial.read(self.serial.in_waiting)
            
            if response == b'\xff\xff\xff':
                print("/!\ \tla requete n'a pas un format valide")
            else:
                beautifull_print(response)
        else:
            print("Aucune réponse du module")

    def open_basic_mode(self):
        GPIO.output(self.pins["M0"], GPIO.LOW)
        GPIO.output(self.pins["M1"], GPIO.LOW)

    def close(self):
        GPIO.cleanup()
        


def beautifull_print(response):
    """Parse the response of the LoRa module and print it in a readable format."""

    print("\n========================== Response ==========================")
    print(response)
    print(f"CMD : {response[0]:02X}")
    print(f"start:{response[1]}\t\tend : {response[2]}")

    print(f"\nADDH : {response[3]}\t\tADDL : {response[4]}")
    print(f"\nNETID : {response[5]}")
    
    reg0 = response[6]
    print(f"\nREG0 = {reg0}")
    print("bits :", f"{reg0:08b}"[:3], "|", f"{reg0:08b}"[3:5], "|", f"{reg0:08b}"[5:])
    print(f"UART baudrate : {UART_BAUD.get((reg0 >> 5) & 0b111, 'Unknown')}")
    print(f"Parité        : {PARITY.get((reg0 >> 3) & 0b11, 'Unknown')}")
    print(f"Air data rate : {AIR_RATE.get(reg0 & 0b111, 'Unknown')}")

    reg1 = response[7]
    print(f"\nREG1 = {reg1}")
    print("bits :", f"{reg1:08b}"[:2], "|", f"{reg1:08b}"[2], "|", f"{reg1:08b}"[2:5], "|", f"{reg1:08b}"[6:])
    print(f"Sub-packet setting : {SUB_PACKET_SETTING.get((reg1 >> 6) & 0b111, 'Unknown')}")
    print(f"RSSI ambient noise : {RSSI_AMBIENT_NOISE.get((reg1 >> 5) & 0b1, 'Unknown')}")
    print(f"Transmitting power : {TRANSMITTING_POWER.get(reg1 & 0b11, 'Unknown')}")
    
    reg3 = response[8]
    print(f"\nREG2 : Chanel {reg3}", "(default)" if reg3==23 else "")
    print(f"frequency = {410.125 + reg3}MHz \t(freq = 410.125 + chanel)")

    reg3 = response[9]
    print(f"\nREG3 = {reg3}")
    print("bits :", f"{reg3:08b}"[0], "|", f"{reg3:08b}"[1], "|", f"{reg3:08b}"[2], "|", f"{reg3:08b}"[3], "|", f"{reg3:08b}"[4], "|", f"{reg3:08b}"[5:])
    print(f"Enable RSSI              : {ENABLE_RSSI.get((reg3 >> 7) & 0b1, 'Unknown')}")
    print(f"Fixed point transmission : {TRANSMISSION_MODE.get((reg3 >> 6) & 0b1, 'Unknown')}")
    print(f"Enable repeater          : {ENABLE_REPEATER.get((reg3 >>  5) & 0b1, 'Unknown')}")
    print(f"LBT enable               : {LBT_ENABLE.get((reg3 >> 4) & 0b1, 'Unknown')}")
    print(f"WOR transceiver control  : {WOR_TRANSCEIVER_CONTROL.get((reg3 >> 3) & 0b1, 'Unknown')}")
    print(f"WOR cycle                : {WOR_CYCLE.get(reg3 & 0b111, 'Unknown')}")

    print(f"\nCRYPT_H : {response[10]}\tCRYPT_L : {response[11]}")
    
    print("==============================================================\n")
    



if __name__ == "__main__":

    # init lora in config mode
    lora = lora33S_config({"M0" : 17, "M1" : 27, "AUX" : 22})

    # read configuration
    print("Asking for configuration to LoRa module, please wait a few seconds...")
    lora.read_configuration()

    # Clean up GPIO
    lora.close()



