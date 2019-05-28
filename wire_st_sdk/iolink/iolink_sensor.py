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


"""iolink_sensor

The iolink_sensor module represents a device capable of connecting to an IOLink
masterboard and sending/receiving data to it.
"""


# IMPORT

import sys
import struct

from serial import SerialException
from serial import SerialTimeoutException

from wire_st_sdk.iolink.iolink_protocol import IOLinkProtocol
from wire_st_sdk.iolink.iolink_device import IOLinkDevice
from wire_st_sdk.utils.wire_st_exceptions import WireInvalidOperationException
from wire_st_sdk.python_utils import lock_for_object


# CLASSES

class IOLinkSensor(IOLinkDevice):
    """IO-Link Sensor class.

    This class manages the commands of a standard IO-Link sensor.
    """

    _SIZE_OF_ENV = 3
    """Number of elements in an environmental domain data."""

    _SIZE_OF_TDM = 6
    """Number of elements in a time domain data."""

    _SIZE_OF_FDM = 4
    """Number of elements per frequency in a frequency domain data."""

    _SIZE_OF_FDM_LINES = 1024
    """Number of elements per frequency in a frequency domain data."""

    _SIZE_OF_FLOAT_bytes = 4
    """Size of floating point numbers in bytes on ARM-32 platforms."""

    _FLOAT_PRECISION = 3
    """Number of digits after the decimal point for sensors' data."""

    def _get_measure(self, measure):
        """Execute a 'get' measure command.

        Args:
            measure (str): The measure command to execute.

        Returns:
            list: A list with the result of the executed measure command.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                while True:
                    self._master._execute(IOLinkProtocol.COMMAND_ICD,
                        IOLinkProtocol.REQUEST_SLAVE)

                    self._master._execute(str(self._position - 1) \
                        + IOLinkProtocol.TERMINATOR_SEQ,
                        IOLinkProtocol.REQUEST_SENSOR_COMMAND)

                    self._master._execute(IOLinkProtocol.COMMAND_MEAS1,
                        IOLinkProtocol.REQUEST_MEASURE_TYPE)

                    if not self._master._execute(measure,
                        IOLinkProtocol.MESSAGE_TRANSMISSION_COMPLETED):

                        self._master._execute(IOLinkProtocol.COMMAND_END,
                        IOLinkProtocol.REQUEST_MOD)
                    else:
                        #print('TRANSMISSION COMPLETED')
                        break

                if not IOLinkProtocol.BYTES_TRANSMISSION:
                    info = self._master.get_answer()[
                        len(IOLinkProtocol.TERMINATOR_SEQ): \
                        - 2 * len(IOLinkProtocol.TERMINATOR_SEQ)
                        - len(IOLinkProtocol.MESSAGE_TRANSMISSION_COMPLETED)]
                else:
                    info = self._master.get_answer()[: \
                        - len(IOLinkProtocol.TERMINATOR_SEQ)
                        - len(IOLinkProtocol.MESSAGE_TRANSMISSION_COMPLETED)]

                #print('SENDING COMMAND END')
                self._master._execute(IOLinkProtocol.COMMAND_END,
                    IOLinkProtocol.REQUEST_MOD)
                #print('COMMAND END SENT')

                return info

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def _bytes_to_floats(self, data, precision=0):
        """Converts an array of bytes to floating point numbers in Little Endian
        order (four bytes per number).

        Args:
            data (str): Input array of bytes that contains the values to convert.
            precision (int): Number of digits after the decimal point.

        Returns:
            list: A list of floating point numbers.
        """
        return [round(
            struct.unpack('<f', struct.pack('cccc', *data[i * 4:i * 4 + 4]))[0],
            precision) \
            for i in range(0, len(data) / 4)]

    def get_env(self):
        """Get environmental data.

        Returns:
            list: A list with Pressure [mbar], Humidity [%], and Temperature [C]
            values.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                while True:
                    info = self._get_measure(IOLinkProtocol.COMMAND_MEAS1_3)

                    if not IOLinkProtocol.BYTES_TRANSMISSION:
                        info = info.split(IOLinkProtocol.TERMINATOR_SEQ)[2:]
                        info = [i for i in info if i != '']
                        if len(info) == IOLinkSensor._SIZE_OF_ENV:
                            break
                    else:
                        if len(info) == IOLinkSensor._SIZE_OF_ENV * \
                            IOLinkSensor._SIZE_OF_FLOAT_bytes:
                            break

                    print('Missing data. Sending again....')

                if not IOLinkProtocol.BYTES_TRANSMISSION:
                    info = list(map(lambda s: float(s), info))
                else:
                    info = self._bytes_to_floats(info, self._FLOAT_PRECISION)

                return info

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def get_tdm(self):
        """Get time domain data.

        Returns:
            list: A two-elements list, with a list of RMS Speed values on X,Y,Z
            axes [mm/s] as the first element, and a list of Peak Acceleration
            values on X,Y,Z axes [m/s2] as the second element.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                while True:
                    info = self._get_measure(IOLinkProtocol.COMMAND_MEAS1_2)

                    if not IOLinkProtocol.BYTES_TRANSMISSION:
                        info = info.split(IOLinkProtocol.TERMINATOR_SEQ)[2:]
                        if len(info) == IOLinkSensor._SIZE_OF_TDM:
                            break
                    else:
                        if len(info) == IOLinkSensor._SIZE_OF_TDM * \
                            IOLinkSensor._SIZE_OF_FLOAT_bytes:
                            break

                    print('Missing data. Sending again....')

                if not IOLinkProtocol.BYTES_TRANSMISSION:
                    info = [list(map(lambda s: float(s), info[0:3])), \
                            list(map(lambda s: float(s), info[3:6]))]
                else:
                    info = self._bytes_to_floats(info, self._FLOAT_PRECISION)
                    info = [info[0:3], info[3:6]]

                return info

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def get_fft(self):
        """Get Fast Fourier Transform of vibration data.

        Returns:
            list: A n-elements list, with each element being a list of four
            values: the first is a frequency [Hz] and the other three are the
            corresponding vibration values on the three axis [m/s2].

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                l = IOLinkSensor._SIZE_OF_FDM_LINES
                n = IOLinkSensor._SIZE_OF_FDM * \
                    IOLinkSensor._SIZE_OF_FLOAT_bytes
                while True:
                    info = self._get_measure(IOLinkProtocol.COMMAND_MEAS1_4)

                    if not IOLinkProtocol.BYTES_TRANSMISSION:
                        info = info.split(IOLinkProtocol.TERMINATOR_SEQ)[2:]
                        if len(info) == l:
                            break
                    else:
                        if len(info) == l * n:
                            break

                    print('Missing data. Sending again...')

                if not IOLinkProtocol.BYTES_TRANSMISSION:
                    for i in range(0, len(info)):
                        info[i] = list(
                            map(lambda s: float(s), info[i].split('\t')[0:-1]))
                else:
                    info = [info[i * n:i * n + n] for i in range(0, l)]
                    info = list(map(lambda s: self._bytes_to_floats(s, \
                        self._FLOAT_PRECISION), info))

                return info

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def _set_parameter(self, parameter, value):
        """Execute a 'set' parameter command.

        Args:
            parameter (str): The parameter command to execute.
            value (str): The parameter value to set.

        Returns:
            bool: True if the parameter has been set correctly, False otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):

                self._master._execute(IOLinkProtocol.COMMAND_ICD,
                    IOLinkProtocol.REQUEST_SLAVE)

                self._master._execute(str(self._position - 1) \
                    + IOLinkProtocol.TERMINATOR_SEQ,
                    IOLinkProtocol.REQUEST_SENSOR_COMMAND)

                self._master._execute(IOLinkProtocol.COMMAND_SET,
                    IOLinkProtocol.REQUEST_PARAMETER_NAME)

                self._master._execute(parameter,
                    IOLinkProtocol.REQUEST_PARAMETER_VALUE)

                self._master._execute(value \
                    + IOLinkProtocol.TERMINATOR_SEQ,
                    IOLinkProtocol.TERMINATOR_SEQ)

                self._master._execute(None, IOLinkProtocol.TERMINATOR_SEQ)

                info = self._master.get_answer()

                self._master._execute(IOLinkProtocol.COMMAND_END,
                    IOLinkProtocol.REQUEST_MOD)

                return True if IOLinkProtocol.MESSAGE_PARAMETER_UPDATED \
                    in info else False

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def set_odr(self, odr):
        """Set accelerometer's output data rate.

        Args:
            odr (:class:`wire_st_sdk.iolink.iolink_protocol.ODR`):
                accelerometer's output data rate.

        Returns:
            bool: True if the parameter has been set correctly, False otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                return self._set_parameter(
                    IOLinkProtocol.COMMAND_ODR,
                    '{:04d}'.format(odr.value))

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def set_fls(self, fls):
        """Set accelerometer's full scale.

        Args:
            fls (:class:`wire_st_sdk.iolink.iolink_protocol.FLS`):
                accelerometer's full scale.

        Returns:
            bool: True if the parameter has been set correctly, False otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                return self._set_parameter(
                    IOLinkProtocol.COMMAND_FLS,
                    '{:02d}'.format(fls.value))

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def set_sze(self, sze):
        """Set accelerometer's input array size for FFT.

        Args:
            sze (:class:`wire_st_sdk.iolink.iolink_protocol.SZE`):
                accelerometer's input array size for FFT.

        Returns:
            bool: True if the parameter has been set correctly, False otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                return self._set_parameter(
                    IOLinkProtocol.COMMAND_SZE,
                    '{:04d}'.format(sze.value))

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def set_sub(self, sub):
        """Set accelerometer's number of subranges.

        Args:
            sub (:class:`wire_st_sdk.iolink.iolink_protocol.SUB`): number of
                subranges.

        Returns:
            bool: True if the parameter has been set correctly, False otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                return self._set_parameter(
                    IOLinkProtocol.COMMAND_SUB,
                    '{:02d}'.format(sub.value))

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def set_acq(self, acq):
        """Set accelerometer's total acquisition time, which is valid for all
        types of analysis.

        Args:
            acq (int): accelerometer's total acquisition time (must be in the
                range [ACQ_MIN..ACQ_MAX]).

        Returns:
            bool: True if the parameter has been set correctly, False otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                self._acq = acq
                return self._set_parameter(
                    IOLinkProtocol.COMMAND_ACQ,
                    '{:05d}'.format(acq))

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def set_ovl(self, ovl):
        """Set accelerometer's overlapping percentage between two consecutive
        FFT analysis.

        Args:
            ovl (int): accelerometer's overlapping percentage between two
                consecutive FFT analysis (must be in the range
                [OVL_MIN..OVL_MAX]).

        Returns:
            bool: True if the parameter has been set correctly, False otherwise.

        Raises:
            'SerialException' or 'SerialTimeoutException' are raised if
                something with the serial communication does not work.
            :exc:`wire_st_sdk.utils.wire_st_exceptions.WireInvalidOperationException`
                is raised if the command has not been executed successfully.
        """
        try:
            with lock_for_object(self._master):
                return self._set_parameter(
                    IOLinkProtocol.COMMAND_OVL,
                    '{:02d}'.format(ovl))

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e
