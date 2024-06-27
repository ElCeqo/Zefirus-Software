'''
Note: This code is meant to be run on a Raspberry Pi for the CubeSat project.


This is the main file for the CubeSat project. This file will be responsible for
running the main loop of the program which will be responsible for
1. reading the ADCS data from the nucleo board
2. communicating with the ground station
3. saving the data to a file
4. processing the data with a Kalman filter.
All the tasks will be run in parallel using processes. (Still to be confirmed)
'''

# Importing the necessary libraries
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), 'modules'))

'''
import custom modules
'''

import EKF
import CommSystem

if __name__ == '__main__':

    '''
    The goal right now is to get the communication system up and running.
    So that it can be tested in the lab
    Start listening to the ground station
    '''


    # Example usage, many registers need to be changed for the actual implementation
    comm_system = CommSystem(bus=0, device=0)
    comm_system.send_data([0x01, 0x02, 0x03])
    received_data = comm_system.listen()
    print(received_data)
    


