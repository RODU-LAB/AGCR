#!/usr/bin/env python3

import can

def setup_can_interface(channel, bitrate):
    """
    Set up the CAN interface with the specified channel and bitrate.
    """
    try:
        bus = can.interface.Bus(channel=channel, bustype='seeedstudio', bitrate=bitrate)
        print(f"CAN interface setup on channel {channel} with bitrate {bitrate}.")
        return bus
    except Exception as e:
        print(f"Error setting up CAN interface: {e}")
        return None

def send_can_message(bus, arbitration_id, data):
    """
    Send a CAN message on the specified bus.
    """
    message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
    try:
        bus.send(message)
        print(f"Message sent: {message}")
    except Exception as e:
        print(f"Error sending message: {e}")

# Setting up CAN interface
channel = '/dev/ttyUSB0'  # Adjust according to your actual channel
bitrate = 1000000  # 1Mbps

bus = setup_can_interface(channel, bitrate)

# Defining the data arrays
close = [0x02, 0x01, 0x20, 0x49, 0x20, 0x00, 0xC8]
open = [0x02, 0x00, 0x20, 0x49, 0x20, 0x00, 0xC8]

if bus:
    while True:
        user_input = input("Enter 1 to send data1, 2 to send data2, or 'q' to quit: ")
        if user_input == '1':
            send_can_message(bus, arbitration_id=0x00000001, data=close)
        elif user_input == '2':
            send_can_message(bus, arbitration_id=0x00000001, data=open)
        elif user_input.lower() == 'q':
            print("Exiting.")
            break
        else:
            print("Invalid input. Please enter 1, 2, or 'q' to quit.")
