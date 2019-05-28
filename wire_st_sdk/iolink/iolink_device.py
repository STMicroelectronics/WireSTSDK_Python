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


"""iolink_device

The iolink_device module represent a device capable of connecting to an IO-Link
masterboard and sending/receiving data to it.
"""


# IMPORT

from serial import SerialException
from serial import SerialTimeoutException

from wire_st_sdk.iolink.iolink_protocol import IOLinkProtocol
from wire_st_sdk.utils.wire_st_exceptions import WireInvalidOperationException
from wire_st_sdk.python_utils import lock_for_object


# CLASSES

class IOLinkDevice(object):
    """IO-Link Device class.

    This class manages the commands of a standard IO-Link device.
    """

    def __init__(self, master, position, id, name=None):
        """Constructor.

        Args:
            master (:class:`wire_st_sdk.iolink.iolink_master.IOLinkMaster`):
                Masterboard object.
            position (int): Device's position according to the enumeration on
                the masterboard.
            id (str): Device's identifier.
            name (str): Device's name.
        """

        self._master = master
        """Masterboard object."""

        self._position = position
        """Device's position according to the enumeration on the masterboard."""

        self._id = id
        """Device's identifier."""

        self._name = name
        """Device's name."""

    def get_position(self):
        """Get the device's position according to the enumeration on the
        masterboard.

        Returns:
            int: The device's position according to the enumeration on the
            masterboard.
        """
        return self._position

    def get_id(self):
        """Get the device's identifier.

        Returns:
            str: The device's identifier.
        """
        return self._id

    def get_name(self):
        """Get the device's name.

        Returns:
            str: The device's name.
        """
        return self._name

    def get_features(self):
        """Get the list of features.

        Returns:
            list: A list with the available features.

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

                self._master._execute(IOLinkProtocol.COMMAND_ID,
                    IOLinkProtocol.MESSAGE_CONNECTED)
                
                info = self._master.get_answer()[
                    len(IOLinkProtocol.TERMINATOR_SEQ): \
                    - len(IOLinkProtocol.MESSAGE_CONNECTED) \
                    - 2 * len(IOLinkProtocol.TERMINATOR_SEQ)]
                info = info.split(IOLinkProtocol.TERMINATOR_SEQ)
                info = list(map(lambda s: s.strip(), info))[:-1]

                self._master._execute(IOLinkProtocol.COMMAND_END,
                    IOLinkProtocol.REQUEST_MOD)

                return info

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e

    def get_firmware(self):
        """Get the version of the firmware.

        Returns:
            str: The version of the firmware as a string.

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

                self._master._execute(IOLinkProtocol.COMMAND_ID,
                    IOLinkProtocol.MESSAGE_CONNECTED)
                
                info = self._master.get_answer()[
                    len(IOLinkProtocol.TERMINATOR_SEQ): \
                    - len(IOLinkProtocol.MESSAGE_CONNECTED) \
                    - 2 * len(IOLinkProtocol.TERMINATOR_SEQ)]
                info = info.split(IOLinkProtocol.TERMINATOR_SEQ)[-1]

                self._master._execute(IOLinkProtocol.COMMAND_END,
                    IOLinkProtocol.REQUEST_MOD)

                return info

        except (SerialException, SerialTimeoutException,
            WireInvalidOperationException) as e:
            raise e
