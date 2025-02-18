import smbus2
import spidev
import RPi.GPIO as GPIO
import time

'''
This class will be responsible for the communication system of the CubeSat.
The communication system will be responsible for sending and receiving data from the ground station.

The OBC is connected via SPI to the radio module.


device address: 0x33
broadcast address: 0x44


In order to retrieve received data from FIFO the user must ensure that ValidHeader, PayloadCrcError, RxDone and
RxTimeout interrupts in the status register RegIrqFlags are not asserted to ensure that packet reception has terminated
successfully (i.e. no flags should be set).


Use packet mode for communication. In packet mode, the module will automatically handle the preamble, sync word, CRC etc.

Ground station uses AX.25 protocol at data link layer. 


Might need to wait for IRQ flags (0x3e ModeReady) to be set before doing anything else.
'''


class CommSystem:
    
    def __init__(self, bus=0, device=0, irq_pin=25, reset_pin=7, cs_pin = 8, device_address=0x33, fifo_address=0x00):
        '''
        Initialize the communication system by connecting to the RFM95/96/97/98(W) 
        Low Power Long Range Transceiver Module.
        '''
        self.bus = bus
        self.device = device
        self.device_address = device_address
        self.fifo_address = fifo_address
        self.cs_pin = cs_pin

        # Set up SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)

        GPIO.cleanup()
        
        # Set up GPIO for IRQ and RESET
        self.irq_pin = irq_pin       
        self.reset_pin = reset_pin
        GPIO.setmode(GPIO.BCM)
        
        try:
            GPIO.setup(self.irq_pin, GPIO.IN)
        except RuntimeWarning:
            print(f"huh")
        
        try:
            GPIO.setup(self.reset_pin, GPIO.OUT)
        except RuntimeWarning:
            print(f"ERRORE")

        
        GPIO.setup(self.cs_pin, GPIO.OUT)
            

        GPIO.output(self.cs_pin, GPIO.HIGH)

        # Reset the LoRa module
        #self.reset()
        
        # Initialize the LoRa module
        self.init_FSK()

    def reset(self):
        '''
        Perform a hardware reset on the LoRa module.
        '''

        GPIO.output(self.reset_pin, GPIO.LOW)

        time.sleep(0.01)
        GPIO.output(self.reset_pin, GPIO.HIGH)
        time.sleep(0.01)

    def init_FSK(self):
        '''
        Initialize the LoRa module with the desired settings.
        '''
        # Basic configuration for FSK mode
        # first set the module to sleep mode and FSK
        self.write_register(0x01, 0x00)

        # Power Amplifier setup (0x09)
        self.write_register(0x09, 0x7F) # Max power

        # Setup transmission parameters RegPacketConfig1 (0x30) and RegPacketConfig2 (0x31)

        '''
        RegPacketConfig1 (0x30)
        bit 7 Packet format: 0 = fixed length, 1 = variable length
        bit 6-5 defines the DC-free encoding/decoding scheme: 00 = none, 01 = Manchester, 10 = Whitening, 11 = reserved
        bit 4 CRC enable: 0 = disabled, 1 = enabled
        bit 3 CRC auto clear: 0 = clear, 1 = no clear
        bit 2-1 Address based filtering in Rx: 00 = none, 01 = node, 10 = node or broadcast
        bit 0 Crc whitening type: 0 = CCITT, 1 = IBM

        We want variable length packets, Whitening, CRC enabled, CRC auto clear, no address filtering, and CCITT whitening 11010000

        RegPacketConfig2 (0x31)
        bit 7 unused
        bit 6 data mode: 0 = continuous, 1 = packet
        bit 5 Io home control: 0 = disabled, 1 = enabled
        bit 4 reserved
        bit 3 beacon on in fixed mode
        bit 2-0 PayloadLength[10:8] Packet length most significant bits

        We want packet mode, Io home control disabled, and a payload length of (to be determined) 01000XXX
        '''

        self.write_register(0x30, 0xD0)
        self.write_register(0x31, 0x40)


        '''
        Set maximum payload length
        '''
        self.write_register(0x32, 0xFF)


        '''
        Set bitrate
        '''
        self.write_register(0x02, 0x68)  # MSB
        self.write_register(0x03, 0x2B)  # LSB

        
        '''
        After setting up the registers, we can set the module to standby mode
        '''

        self.write_register(0x01, 0x01)



    def write_register(self, address, value):
        '''
        Write a value to a register.
        '''
        GPIO.output(self.cs_pin, GPIO.LOW)
        self.spi.xfer2([address | 0x80, value])
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def read_register(self, address):
        '''
        Read a value from a register.
        '''
        GPIO.output(self.cs_pin, GPIO.LOW)
        response = self.spi.xfer2([address & 0x7F, 0x00])
        GPIO.output(self.cs_pin, GPIO.HIGH)
        return response[1]

    def send_data(self, data):
        '''
        Send data to the ground station.
        '''
        # Set the LoRa module to standby mode
        self.write_register(0x01, 0x01)
        
        # Write data to the FIFO
        self.write_register(0x00, 0x00)
        for byte in data:
            self.write_register(0x00, byte)
        
        # Set the LoRa module to transmit mode
        self.write_register(0x01, 0x03)

        # Wait for the transmission to finish (irq flag set to 1)
        while True:
            if GPIO.input(self.irq_pin) == GPIO.HIGH:
                irq_flags = self.read_register(0x3F)
                if irq_flags & 0x08:
                    return

    def listen(self):
        '''
        Listen for incoming data.
        '''
        # Set to receive mode
        self.write_register(0x01, 0x05)
        
        while True:
            if GPIO.input(self.irq_pin) == GPIO.HIGH:
                irq_flags = self.read_register(0x3F)
                if irq_flags & 0x04: # RX done
                    # Read the received data
                    length = self.read_register(0x13)
                    data = []
                    for _ in range(length):
                        data.append(self.read_register(0x00))
                    return data




if __name__ == '__main__':
    comm = CommSystem()
    comm.send_data([0x01, 0x02, 0x03, 0x04])