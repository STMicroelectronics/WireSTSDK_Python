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


"""iolink_master

The iolink_master module is responsible for managing IO-Link capable devices
connected to an iolink masterboard and allocating the needed resources.
"""


# IMPORT

from abc import ABCMeta
from abc import abstractmethod
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
from serial import SerialException
from serial import SerialTimeoutException
import time
from datetime import datetime

from wire_st_sdk.iolink.iolink_protocol import IOLinkProtocol
from wire_st_sdk.iolink.iolink_sensor import IOLinkSensor
from wire_st_sdk.utils.wire_st_exceptions import WireSTInvalidOperationException
from wire_st_sdk.utils.python_utils import lock
from wire_st_sdk.utils.python_utils import lock_for_object


# CLASSES

class IOLinkMaster(object):
    """IO-Link Master class.

    This class manages the communication between the host and the masterboard.
    """

    _NUMBER_OF_DEVICES = 4
    """Number of devices that can be connected to the masterboard."""

    _CONFIGURATION_REGISTERS_STRING = '096,248,033,122,122,122,122,' \
        + IOLinkProtocol.TERMINATOR_SEQ
    """Configuration string for the masterboard's registers."""

    _NUMBER_OF_THREADS = 5
    """Number of threads to be used to notify the listeners."""

    _SERIAL_PORT_RW_DELAY_s = 0.1
    """Delay time in seconds between two consecutive reads/writes on the serial
    port."""

    _FDM_ATTEMPTS = 10
    """Number of attempts to receive a line of frequency domain data."""

    def __init__(self, serial_port):
        """Constructor.

        Args:
            serial_port (Serial): Serial Port object. Refer to
            `Serial <https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial>`_
            for more information.
        """
    
        self._serial_port = serial_port
        """Serial Port object."""

        self._answer = None
        """Last answer received on the serial port when executing a command."""

        self._status = IOLinkMasterStatus.INIT
        """Status."""

        self._thread_pool = ThreadPoolExecutor(IOLinkMaster._NUMBER_OF_THREADS)
        """Pool of thread used to notify the listeners."""

        self._listeners = []
        """List of listeners to the master changes.
        It is a thread safe list, so a listener can subscribe itself through a
        callback."""

        self._devices = [None] * self._NUMBER_OF_DEVICES
        """List of devices by position."""

        self._devices_id_position_map = {}
        """Dictionary that maps devices' identifier and devices' position."""

        # Updating status.
        self._update_master_status(IOLinkMasterStatus.IDLE)

    def _update_master_status(self, new_status):
        """Update the status of the master.

        Args:
            new_status (:class:`wire_st_sdk.iolink.master.IOLinkMasterStatus`):
            New status.
        """
        old_status = self._status
        self._status = new_status
        for listener in self._listeners:
            # Calling user-defined callback.
            self._thread_pool.submit(
                listener.on_status_change(
                    self, new_status.value, old_status.value))

    def _update_master_device(self, device_id, device_position):
        """Update about the new device found.

        Args:
            device_id (str): New device found.
            device_position (int): Position of the new device found.
        """
        for listener in self._listeners:
            # Calling user-defined callback.
            self._thread_pool.submit(
                listener.on_device_found(
                    self, device_id, device_position))

    def _execute(self, command=None, expected_answer=None):
        """Execute a command and check the status.

        Args:
            command (str): Command to perform.
            expected_answer (str): Expected answer to check after performing the
            command.

        Returns:
            bool: True if the command has been executed correctly, False
            otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
            something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireSTInvalidOperationException`
            is raised if the command has not been executed successfully.
        """
        try:
            # Encoding string to bytes (Python 3).
            command = command.encode('utf-8') if command else command
            expected_answer = expected_answer.encode('utf-8') if expected_answer \
                else expected_answer

            with lock_for_object(self):

                # Delay.
                time.sleep(self._SERIAL_PORT_RW_DELAY_s)

                # Blocking write.
                if command:
                    #print('---> \"%s\"' % (command[:-2]))
                    self._serial_port.write(command)
                    self._serial_port.flush()

                # Delay.
                time.sleep(self._SERIAL_PORT_RW_DELAY_s)

                # Blocking read.
                if expected_answer:
                    if not (command == IOLinkProtocol.COMMAND_MEAS1_4 and \
                        IOLinkProtocol.BYTES_TRANSMISSION):
                        self._answer = b''
                        while True:
                            self._answer += \
                                self._serial_port.read_until(expected_answer)
                            if expected_answer in self._answer:
                                return True
                            if IOLinkProtocol.MESSAGE_SENSOR_FAILED.encode('utf-8') \
                                in self._answer \
                                or IOLinkProtocol.MESSAGE_PARAMETER_UPDATED.encode('utf-8') \
                                in self._answer:
                                return False
                    else:
                        USE_ACK = False
                        self._answer = b''
                        n = IOLinkSensor._SIZE_OF_FDM * \
                            IOLinkSensor._SIZE_OF_FLOAT_bytes
                        attempts = self._FDM_ATTEMPTS
                        while attempts:
                            data = self._serial_port.read(n)
                            if len(data) == n:
                                attempts = self._FDM_ATTEMPTS
                                self._answer += data
                                if USE_ACK:
                                    self._serial_port.write(
                                        IOLinkProtocol.ACK.encode('utf-8'))
                                    self._serial_port.flush()
                            else:
                                if USE_ACK:
                                    self._serial_port.write(
                                        IOLinkProtocol.NACK.encode('utf-8'))
                                    self._serial_port.flush()
                                    attempts -= 1
                                    #print('Trying again...(%d)' % (attempts))
                                else:
                                    return False
                            # Constant '1024' to be substituted with number of
                            # lines parameter.
                            if len(self._answer) == 1024 * n:
                                while True:
                                    self._answer += \
                                        self._serial_port.read_until(
                                            expected_answer)
                                    if expected_answer in self._answer:
                                        return True
                        return False

                # Blocking read.
                # self._answer = b''
                # if expected_answer:
                #     while expected_answer not in self._answer:
                #         self._answer += self._serial_port.read_until(expected_answer)
                #         if IOLinkProtocol.MESSAGE_SENSOR_FAILED.encode('utf-8') \
                #             in self._answer \
                #             or IOLinkProtocol.MESSAGE_PARAMETER_UPDATED.encode('utf-8') \
                #             in self._answer:
                #             # Sensor has already been rebooted.
                #             return False

                # # Non-blocking read.
                # self._answer = self._serial_port.read(1)
                # while expected_answer not in self._answer:
                #     while (self._serial_port.in_waiting > 0):
                #         self._answer += self._serial_port.read(self._serial_port.in_waiting)
                #     if IOLinkProtocol.MESSAGE_SENSOR_FAILED.encode('utf-8') \
                #         in self._answer \
                #         or IOLinkProtocol.MESSAGE_PARAMETER_UPDATED.encode('utf-8') \
                #         in self._answer:
                #         return False

                return True

        except (SerialException, SerialTimeoutException,
            WireSTInvalidOperationException) as e:
            raise e

    def _get_answer(self):
        """Get the last answer received on the serial port when executing a
        command.

        Returns:
            bytes: The last answer received on the serial port when executing a
            command.
        """
        return self._answer

    def connect(self):
        """Start the communication between the masterboard and the host to which
        it is connected.
        
        Returns:
            str: The status of the connection.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
            something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireSTInvalidOperationException`
            is raised if the command has not been executed successfully.
        """
        try:

            with lock_for_object(self):

                # Updating status.
                self._update_master_status(IOLinkMasterStatus.CONNECTING)

                # Opening the serial port.

                if not self._serial_port.is_open:
                    self._serial_port.open()

                # Configuring all devices.

                self._execute(IOLinkProtocol.COMMAND_START,
                    IOLinkProtocol.REQUEST_MOD)

                connection_answer = IOLinkProtocol.TERMINATOR_SEQ.encode('utf-8') \
                    + self._answer[: \
                    - len(IOLinkProtocol.REQUEST_MOD) \
                    - len(IOLinkProtocol.TERMINATOR_SEQ)]

                for device_position in range(self._NUMBER_OF_DEVICES):

                    self._execute(IOLinkProtocol.COMMAND_ICM,
                        IOLinkProtocol.REQUEST_IC)

                    self._execute(str(device_position) \
                        + IOLinkProtocol.TERMINATOR_SEQ,
                        IOLinkProtocol.REQUEST_OPM)

                    self._execute(IOLinkProtocol.COMMAND_WRS,
                        IOLinkProtocol.REQUEST_RVAL)

                    self._execute(self._CONFIGURATION_REGISTERS_STRING,
                        IOLinkProtocol.MESSAGE_PROGRAMMING_DONE)

                    self._execute(IOLinkProtocol.COMMAND_END,
                        IOLinkProtocol.REQUEST_MOD)

                # Clearing devices' maps.

                self._devices = [None] * self._NUMBER_OF_DEVICES
                self._devices_id_position_map.clear()

                # Getting all devices' identifiers.

                for device_position in range(self._NUMBER_OF_DEVICES):

                    self._execute(IOLinkProtocol.COMMAND_ICD,
                        IOLinkProtocol.REQUEST_SLAVE)

                    self._execute(str(device_position) \
                        + IOLinkProtocol.TERMINATOR_SEQ,
                        IOLinkProtocol.REQUEST_SENSOR_COMMAND)

                    self._timeout = self._serial_port.timeout
                    self._serial_port.timeout = 5

                    self._execute(IOLinkProtocol.COMMAND_GET_1,
                        IOLinkProtocol.TERMINATOR_SEQ)

                    self._serial_port.timeout = self._timeout

                    self._execute(None, IOLinkProtocol.TERMINATOR_SEQ)

                    # Handling sensor error.
                    if IOLinkProtocol.MESSAGE_SENSOR_FAILED.encode('utf-8') \
                        in self._get_answer() \
                        or IOLinkProtocol.MESSAGE_PARAMETER_UPDATED.encode('utf-8') \
                        in self._get_answer():
                        # Sensor has already been rebooted.
                        #print('Rebooting sensor...')
                        #print('---> "MCU" KO: \"%s\"' % (self._answer[:-2]))
                        self._execute(IOLinkProtocol.COMMAND_END,
                            IOLinkProtocol.REQUEST_MOD)
                        continue

                    # Getting device's identifier, build devices' map, and
                    # waiting for transmission completed.
                    device_id = self._get_answer()[:
                        - len(IOLinkProtocol.TERMINATOR_SEQ)
                        ].strip().decode('utf-8')

                    self._devices[device_position] = device_id

                    self._devices_id_position_map[device_id] = \
                        device_position + 1

                    self._execute(None, IOLinkProtocol.TERMINATOR_SEQ)

                    self._execute(None,
                        IOLinkProtocol.MESSAGE_TRANSMISSION_COMPLETED)

                    self._execute(IOLinkProtocol.COMMAND_END,
                        IOLinkProtocol.REQUEST_MOD)

                    # Updating about new device found.
                    self._update_master_device(device_id, device_position + 1)

                self._answer = connection_answer

                # Updating status.
                self._update_master_status(IOLinkMasterStatus.CONNECTED)

                return self._get_answer()

        except (SerialException, SerialTimeoutException,
            WireSTInvalidOperationException) as e:
            raise e

    def disconnect(self):
        """End the communication between the masterboard and the host to which
        it is connected.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
            something with the serial communication does not work.
        """
        try:
            if not self.is_connected():
                return

            with lock_for_object(self):

                # Updating status.
                self._update_master_status(IOLinkMasterStatus.DISCONNECTING)

                # Clearing devices' maps.
                self._devices = [None] * self._NUMBER_OF_DEVICES
                self._devices_id_position_map.clear()

                # Closing the serial port.
                if self._serial_port.is_open:
                    self._serial_port.close()

                # Updating status.
                self._update_master_status(IOLinkMasterStatus.IDLE)

        except (SerialException, SerialTimeoutException) as e:
            raise e

    def is_connected(self):
        """Check whether the master is connected.

        Returns:
            bool: True if the master is connected, False otherwise.
        """
        return self._get_answer() == IOLinkMasterStatus.CONNECTED

    def get_device(self, device_id, device_name=None):
        """Configure and get a connected device.

        Args:
            device_id (str): Device's identifier.
            device_name (str): Device's name.

        Returns:
            :class:`wire_st_sdk.iolink.iolink_device.IOLinkDevice`: IOLink
            connected device if the device identifier is correct, None
            otherwise.

        Raises:
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireSTInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        # Creating the device.
        try:
            with lock_for_object(self):
                if self._get_answer() == None:
                    msg = '\nBefore trying to get a device, connect to the '\
                        'masterboard.'
                    raise WireSTInvalidOperationException(msg)

                if device_id in self._devices_id_position_map:
                    device_position = self._devices_id_position_map[device_id]
                    return IOLinkSensor(
                        self, device_position, device_id, device_name)

                return None

        except WireSTInvalidOperationException as e:
            raise e

    def get_device_by_position(self, device_position, device_name=None):
        """Configure and get a connected device.

        Args:
            device_position (int): Device's position according to the
            enumeration on the masterboard.
            device_name (str): Device's name.

        Returns:
            :class:`wire_st_sdk.iolink.iolink_device.IOLinkDevice`: IOLink
            connected device if there is a device connected at the given device
            position, None otherwise.

        Raises:
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireSTInvalidOperationException`
            is raised if the command has not been executed successfully.
            :exc:`ValueError` if the device position is not allowed.
        """
        # Creating the device.
        try:
            with lock_for_object(self):
                if self._get_answer() == None:
                    msg = '\nBefore trying to get a device, connect to the '\
                        'masterboard.'
                    raise WireSTInvalidOperationException(msg)

                if device_position not in range(1, self._NUMBER_OF_DEVICES + 1):
                    raise ValueError('Device\'s position must be in range ' \
                        '[%d..%d].' % (1, self._NUMBER_OF_DEVICES))

                if device_position in range(1, self._NUMBER_OF_DEVICES + 1):
                    device_id = self._devices[device_position - 1]
                    return IOLinkSensor(
                        self, device_position, device_id, device_name)

                return None

        except WireSTInvalidOperationException as e:
            raise e

    def get_port(self):
        """Get the serial port of the masterboard.

        Returns:
            Serial: Serial Port object. Refer to
            `Serial <https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial>`_
            for more information.
        """
        return self._serial_port

    def add_listener(self, listener):
        """Add a listener.
        
        Args:
            listener (:class:`wire_st_sdk.iolink.MasterListener`): Listener to
            be added.
        """
        if listener is not None:
            with lock(self):
                if not listener in self._listeners:
                    self._listeners.append(listener)

    def remove_listener(self, listener):
        """Remove a listener.

        Args:
            listener (:class:`wire_st_sdk.iolink.MasterListener`): Listener to
            be removed.
        """
        if listener is not None:
            with lock(self):
                if listener in self._listeners:
                    self._listeners.remove(listener)


class IOLinkMasterStatus(Enum):
    """Status of the masterboard."""

    INIT = 'INIT'
    """Dummy initial status."""

    IDLE = 'IDLE'
    """Waiting for a connection."""

    CONNECTING = 'CONNECTING'
    """Opening a connection with the host."""

    CONNECTED = 'CONNECTED'
    """Connected to the host."""

    DISCONNECTING = 'DISCONNECTING'
    """Closing the connection to the host."""


# INTERFACES

class IOLinkMasterListener(object):
    """Interface used by the :class:`wire_st_sdk.iolink.IOLinkMaster` class to
    notify changes of a masterboard's status.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def on_status_change(self, masterboard, new_status, old_status):
        """To be called whenever a masterboard changes its status.

        Args:
            masterboard (:class:`wire_st_sdk.iolink.IOLinkMaster`): Masterboard
            that has changed its status.
            new_status (:class:`wire_st_sdk.iolink.IOLinkMasterStatus`): New
            status.
            old_status (:class:`wire_st_sdk.iolink.IOLinkMasterStatus`): Old
            status.

        Raises:
            'NotImplementedError' is raised if the method is not implemented.
        """
        raise NotImplementedError('You must define "on_status_change()" to use '
            'the "IOLinkMasterListener" class.')

    @abstractmethod
    def on_device_found(self, masterboard, device_id, device_position):
        """To be called whenever a masterboard finds a new device connected.

        Args:
            masterboard (:class:`wire_st_sdk.iolink.IOLinkMaster`): Masterboard
            that has found a new device.
            device_id (str): New device found.
            device_position (int): Position of the new device found.

        Raises:
            'NotImplementedError' is raised if the method is not implemented.
        """
        raise NotImplementedError('You must define "on_device_found()" to use '
            'the "IOLinkMasterListener" class.')
