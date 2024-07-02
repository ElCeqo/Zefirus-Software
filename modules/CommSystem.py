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

'''


class CommSystem:
    
    def __init__(self, bus=0, device=0, irq_pin=25, reset_pin=7, device_address=0x33, fifo_address=0x00):
        '''
        Initialize the communication system by connecting to the RFM95/96/97/98(W) 
        Low Power Long Range Transceiver Module.
        '''
        self.bus = bus
        self.device = device
        self.device_address = device_address
        self.fifo_address = fifo_address

        # Set up SPI
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)

        # Set up GPIO for IRQ and RESET
        self.irq_pin = irq_pin            # still to be defined
        self.reset_pin = reset_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.irq_pin, GPIO.IN)
        GPIO.setup(self.reset_pin, GPIO.OUT)

        # Reset the LoRa module
        self.reset()
        
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
        # first set the module to sleep mode
        self.RegOpMode = 0x01
        self.write_register(self.RegOpMode, 0x00)
        # set the module to FSK mode, can be done only after the module is in sleep mode
        self.write_register(0x01, 0x00)

        # Carrier frequency registers setup (0x06, 0x07, 0x08)
        self.write_register(0x06, 0x6C) # Frequency (example: 915 MHz)
        self.write_register(0x07, 0x80)
        self.write_register(0x08, 0x00)

        # Power Amplifier setup (0x09)
        self.write_register(0x09, 0x7F) # Max power

    def write_register(self, address, value):
        '''
        Write a value to a register.
        '''
        self.spi.xfer2([address | 0x80, value])

    def read_register(self, address):
        '''
        Read a value from a register.
        '''
        response = self.spi.xfer2([address & 0x7F, 0x00])
        return response[1]

    def send_data(self, data):
        '''
        Send data to the ground station.
        '''
        # Set the LoRa module to standby mode
        self.write_register(0x01, 0x81)
        
        # Write data to the FIFO
        self.write_register(0x00, 0x00)
        for byte in data:
            self.spi.xfer2([0x00 | 0x80, byte])
        
        # Set the length of the payload
        self.write_register(0x22, len(data))
        
        # Set the LoRa module to transmit mode
        self.write_register(0x01, 0x83)

    def listen(self):
        '''
        Listen for incoming data.
        '''
        # Set the LoRa module to receive continuous mode
        self.write_register(0x01, 0x85)
        
        while True:
            if GPIO.input(self.irq_pin) == GPIO.HIGH:
                irq_flags = self.read_register(0x12)
                if irq_flags & 0x40: # RX done
                    # Read the received data
                    self.write_register(0x0D, 0x00)
                    length = self.read_register(0x13)
                    data = []
                    for _ in range(length):
                        data.append(self.read_register(0x00))
                    # Clear IRQ flags
                    self.write_register(0x12, 0xFF)
                    return data


