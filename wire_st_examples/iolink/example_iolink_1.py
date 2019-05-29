#!/usr/bin/env python

################################################################################
# COPYRIGHT(c) 2018 STMicroelectronics                                         #
#                                                                              #
# Redistribution and use in source and binary forms, with or without           #
# modification, are permitted provided that the following conditions are met:  #
#   1. Redistributions of source code must retain the above copyright notice,  #
#      this list of conditions and the following disclaimer.                   #
#   2. Redistributions in binary form must reproduce the above copyright       #
#      notice, this list of conditions and the following disclaimer in the     #
#      documentation and/or other materials provided with the distribution.    #
#   3. Neither the name of STMicroelectronics nor the names of its             #
#      contributors may be used to endorse or promote products derived from    #
#      this software without specific prior written permission.                #
#                                                                              #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE    #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR          #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS     #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)      #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   #
# POSSIBILITY OF SUCH DAMAGE.                                                  #
################################################################################

################################################################################
# Author:  Davide Aliprandi, STMicroelectronics                                #
################################################################################


# DESCRIPTION
#
# This application example shows how to connect IO-Link devices to a Linux
# gateway and to get data from them.


# IMPORT

from __future__ import print_function
import sys
import os
import time
import getopt
import json
import logging
from enum import Enum
import threading
import serial
from serial import SerialException
from serial import SerialTimeoutException

import wire_st_sdk.iolink.iolink_protocol as iolink_protocol
from wire_st_sdk.iolink.iolink_master import IOLinkMaster
from wire_st_sdk.iolink.iolink_master import IOLinkMasterListener
from wire_st_sdk.iolink.iolink_device import IOLinkDevice
from wire_st_sdk.utils.wire_st_exceptions import WireInvalidOperationException


# PRECONDITIONS
#
# In case you want to modify the SDK, clone the repository and add the location
# of the "WireSTSDK_Python" folder to the "PYTHONPATH" environment variable.
#
# On Linux:
#   export PYTHONPATH=/home/<user>/WireSTSDK_Python


# CONSTANTS

# Presentation message.
INTRO = """###################
# IO-Link Example #
###################"""

# IO-Link settings.
SERIAL_PORT_NAME = '/dev/ttyUSB0'
SERIAL_PORT_BAUDRATE_bs = 230400
SERIAL_PORT_TIMEOUT_s = 5

# Devices' identifier.
IOT_DEVICE_1_ID = '393832383035511900430037'
IOT_DEVICE_2_ID = '3938323830355119003B0038'


# FUNCTIONS

#
# Printing intro.
#
def print_intro():
    print('\n' + INTRO + '\n')


# CLASSES

#
# Implementation of the interface used by the IOLinkMaster class to notify the
# status of the connection.
#
class MyIOLinkMasterListener(IOLinkMasterListener):

    #
    # To be called whenever a masterboard changes its status.
    #
    # @param masterboard IOLinkMaster instance that has changed its status.
    # @param new_status New masterboard status.
    # @param old_status Old masterboard status.
    #
    def on_status_change(self, masterboard, new_status, old_status):
        print('Masterboard on port %s from %s to %s.' %
            (masterboard.get_port().port, str(old_status), str(new_status)))

    #
    # To be called whenever a masterboard finds a new device connected.
    #
    # @param masterboard (IOLinkMaster): Masterboard that has found a new device.
    # @param device_id (str): New device found.
    # @param device_position (int): Position of the new device found.
    #
    def on_device_found(self, masterboard, device_id, device_position):
        print('Masterboard on port %s has found device %s on position %d.' %
            (masterboard.get_port().port, device_id, device_position))  


# MAIN APPLICATION

#
# Main application.
#
def main(argv):

    # Printing intro.
    print_intro()

    try:
        # Initializing Serial Port.
        serial_port = serial.Serial()
        serial_port.port = SERIAL_PORT_NAME
        serial_port.baudrate = SERIAL_PORT_BAUDRATE_bs
        serial_port.parity = serial.PARITY_NONE
        serial_port.stopbits = serial.STOPBITS_ONE
        serial_port.bytesize = serial.EIGHTBITS
        serial_port.timeout = SERIAL_PORT_TIMEOUT_s
        serial_port.write_timeout = None

        # Initializing an IO-Link Masterboard and connecting it to the host.
        print('\nInitializing Masterboard on port %s with a baud rate of %d ' \
            '[b/s]...' % (serial_port.port, serial_port.baudrate))
        master = IOLinkMaster(serial_port)
        master_listener = MyIOLinkMasterListener()
        master.add_listener(master_listener)
        status = master.connect()

        # Initializing IO-Link Devices.
        print('Initializing IO-Link Devices...')
        devices = []
        devices.append(master.get_device(IOT_DEVICE_1_ID))
        devices.append(master.get_device(IOT_DEVICE_2_ID))

        # Checking setup.
        for device in devices:
            if not device:
                print('IO-Link setup incomplete. Exiting...\n')
                sys.exit(0)

        # IO-Link setup complete.
        print('\nIO-Link setup complete.\n')

        # Getting information about devices.
        for device in devices:
            print('Device %d:' % (device.get_position()))
            print('\tDevice Id:\n\t\t\"%s\"' % (device.get_id()))
            print('\tFirmware:\n\t\t\"%s\"' % (device.get_firmware()))
            print('\tFeatures:\n\t\t%s' % (device.get_features()))
        print()

        # Setting devices' parameters.
        # odr = iolink_protocol.ODR.ODR_6660
        # fls = iolink_protocol.FLS.FLS_16
        # sze = iolink_protocol.SZE.SZE_1024
        # sub = iolink_protocol.SUB.SUB_64
        # acq = iolink_protocol.ACQ_MIN
        # ovl = iolink_protocol.OVL_MIN
        # for device in devices:
        #     print('Device %d:' % (device.get_position()))
        #     print('\tSetting ODR to \"%s\"...' % (odr.value), end='')
        #     sys.stdout.flush()
        #     print('Done' if device.set_odr(odr) else 'Error')
        #     print('\tSetting FLS to \"%s\"...' % (fls.value), end='')
        #     sys.stdout.flush()
        #     print('Done' if device.set_fls(fls) else 'Error')
        #     print('\tSetting SZE to \"%s\"...' % (sze.value), end='')
        #     sys.stdout.flush()
        #     print('Done' if device.set_sze(sze) else 'Error')
        #     print('\tSetting SUB to \"%s\"...' % (sub.value), end='')
        #     sys.stdout.flush()
        #     print('Done' if device.set_sub(sub) else 'Error')
        #     print('\tSetting ACQ to \"%s\"...' % (acq), end='')
        #     sys.stdout.flush()
        #     print('Done' if device.set_acq(acq) else 'Error')
        #     print('\tSetting OVL to \"%s\"...' % (ovl), end='')
        #     sys.stdout.flush()
        #     print('Done' if device.set_ovl(ovl) else 'Error')
        # print()

        # Getting measures from devices.
        for device in devices:
            print('Device %d:' % (device.get_position()))
            print('\tEnvironmental data (P[mbar], H[%%], T[C]):\n\t\t%s' \
                % (device.get_env()))
            print('\tTime domain data, RMS Speed [mm/s] and Peak Acceleration' \
                ' [m/s2]:\n\t\t%s' % (device.get_tdm()))
            print('\tFast Fourier Transform of vibration data [m/s2]:')
            i = 0
            for l in device.get_fft():
                print('\t\t%d) %s' % (i, l))
                i += 1
        print()

        master.disconnect()
        sys.exit(0)


    except (WireInvalidOperationException, \
        SerialException, SerialTimeoutException, \
        ValueError) as e:
        print(e)
        master.disconnect()
        print('Exiting...\n')
        sys.exit(0)
    except KeyboardInterrupt:
        try:
            master.disconnect()
            print('\nExiting...\n')
            sys.exit(0)
        except SystemExit:
            os._exit(0)


if __name__ == "__main__":

    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
