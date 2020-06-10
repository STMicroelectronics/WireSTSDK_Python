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
from wire_st_sdk.utils.wire_st_exceptions import WireSTInvalidOperationException
from wire_st_sdk.utils.python_utils import lock_for_object


# CLASSES

class IOLinkDevice(object):
    """IO-Link Device class.

    This class manages the commands of a standard IO-Link device.
    """

    def __init__(self, master, position, id, name=None):
        """Constructor.

        :param master: Masterboard object.
        :type master: :class:`wire_st_sdk.iolink.iolink_master.IOLinkMaster`

        :param position: Device's position according to the enumeration on the
            masterboard.
        :type position: int

        :param id: Device's identifier.
        :type id: str

        :param name: Device's name.
        :type name: str
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

        :returns: The device's position according to the enumeration on the
            masterboard.
        :rtype: int
        """
        return self._position

    def get_id(self):
        """Get the device's identifier.

        :returns: The device's identifier.
        :rtype: str
        """
        return self._id

    def get_name(self):
        """Get the device's name.

        :returns: The device's name.
        :rtype: str
        """
        return self._name

    def get_features(self):
        """Get the list of features.

        :returns: A list with the available features.
        :rtype: list

        :raises SerialException, SerialTimeoutException: are raised if something
            with the serial communication does not work.
        :raises WireSTInvalidOperationException: is raised if the command has
            not been executed successfully.
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
                
                info = self._master._get_answer().decode('utf-8')[
                    len(IOLinkProtocol.TERMINATOR_SEQ): \
                    - len(IOLinkProtocol.MESSAGE_CONNECTED) \
                    - 2 * len(IOLinkProtocol.TERMINATOR_SEQ)]
                info = info.split(IOLinkProtocol.TERMINATOR_SEQ)
                info = list(map(lambda s: s.strip(), info))[:-1]

                self._master._execute(IOLinkProtocol.COMMAND_END,
                    IOLinkProtocol.REQUEST_MOD)

                return info

        except (SerialException, SerialTimeoutException,
            WireSTInvalidOperationException) as e:
            raise e

    def get_firmware(self):
        """Get the version of the firmware.

        :returns: The version of the firmware as a string.
        :rtype: str

        :raises SerialException, SerialTimeoutException: are raised if something
            with the serial communication does not work.
        :raises WireSTInvalidOperationException: is raised if the command has
            not been executed successfully.
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
                
                info = self._master._get_answer().decode('utf-8')[
                    len(IOLinkProtocol.TERMINATOR_SEQ): \
                    - len(IOLinkProtocol.MESSAGE_CONNECTED) \
                    - 2 * len(IOLinkProtocol.TERMINATOR_SEQ)]
                info = info.split(IOLinkProtocol.TERMINATOR_SEQ)[-1]

                self._master._execute(IOLinkProtocol.COMMAND_END,
                    IOLinkProtocol.REQUEST_MOD)

                return info

        except (SerialException, SerialTimeoutException,
            WireSTInvalidOperationException) as e:
            raise e
