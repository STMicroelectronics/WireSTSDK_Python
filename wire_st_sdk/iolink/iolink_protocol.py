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


"""iolink_protocol

The iolink_protocol module contains definitions related to the IO-Link protocol.
"""


# IMPORT

from enum import Enum


# COMMAND STRINGS

class IOLinkProtocol(object):
    """This class lists the protocol's commands used for IO-Link communication.
    """

    # Data transmission.

    BYTES_TRANSMISSION = True
    """If True, transmission of data is by bytes; by characters otherwise."""


    # Common sequences.

    TERMINATOR_SEQ = '\r\n'
    """Terminator sequence."""
    ACK = '1'
    """Acknowledgement."""
    NACK = '0'
    """Non-Acknowledgement."""


    # Commands to the masterboard.

    COMMAND_START = 'START' + TERMINATOR_SEQ
    """Open RS485 communication."""
    COMMAND_WRC = 'WR_C' + TERMINATOR_SEQ
    """L6360 write curent mode."""
    COMMAND_RDC = 'RD_C' + TERMINATOR_SEQ
    """L6360 read curent mode."""
    COMMAND_WRS = 'WR_S' + TERMINATOR_SEQ
    """L6360 write sequential mode."""
    COMMAND_RDS = 'RD_S' + TERMINATOR_SEQ
    """L6360 read sequential mode."""
    COMMAND_ICM = 'MASTER' + TERMINATOR_SEQ
    """Master settings mode."""
    COMMAND_ICD = 'DEVICE' + TERMINATOR_SEQ
    """IO-Link communication mode."""
    COMMAND_NEW = 'COMMAND NEW' + TERMINATOR_SEQ
    """Continue programming on the same node."""
    COMMAND_END = 'COMMAND END' + TERMINATOR_SEQ
    """Close actual programming mode and change node."""
    COMMAND_ID = 'IDS' + TERMINATOR_SEQ
    """Get sensors list."""
    COMMAND_SP = 'PRM' + TERMINATOR_SEQ
    """Get sensor's parameter."""
    COMMAND_SP1 = 'SENSOR ACC PRM' + TERMINATOR_SEQ
    """Get accelerometer's data."""
    COMMAND_SP2 = 'SENSOR ENV PRM' + TERMINATOR_SEQ
    """Get evironmental data."""
    COMMAND_MEAS1 = 'MSR' + TERMINATOR_SEQ
    """Request for measurement."""
    COMMAND_MEAS1_1 = 'RAW' + TERMINATOR_SEQ
    """RAW data."""
    COMMAND_MEAS1_2 = 'TDM' + TERMINATOR_SEQ
    """Time domain data."""
    COMMAND_MEAS1_3 = 'ENV' + TERMINATOR_SEQ
    """Environmental data."""
    COMMAND_MEAS1_4 = 'FFT' + TERMINATOR_SEQ
    """Fast Fourier Transform data."""
    COMMAND_GET = 'STS' + TERMINATOR_SEQ
    """Get sensor's status, vibration peak, RMS, or vibration overtoken."""
    COMMAND_GET_1 = 'MCU' + TERMINATOR_SEQ
    """Microcontroller's identifier."""
    COMMAND_SET1 = 'SENSOR CNF' + TERMINATOR_SEQ
    """Sensor configuration parameters."""
    COMMAND_SET1_1 = 'SENSOR ACC CNF' + TERMINATOR_SEQ
    """Sensor configuration accelerometer parameters."""
    COMMAND_SET1_2 = 'SENSOR ACC CPT' + TERMINATOR_SEQ
    """Sensor configuration computation parameters."""
    COMMAND_SET = 'SET' + TERMINATOR_SEQ
    """Set sensor parameter."""
    COMMAND_ODR = 'ODR' + TERMINATOR_SEQ
    """Set accelerometer's output data rate."""
    COMMAND_FLS = 'FLS' + TERMINATOR_SEQ
    """Set accelerometer's full scale."""
    COMMAND_SZE = 'SZE' + TERMINATOR_SEQ
    """Set accelerometer's input array size for FFT."""
    COMMAND_SUB = 'SUB' + TERMINATOR_SEQ
    """Set accelerometer's number of subranges."""
    COMMAND_ACQ = 'ACQ' + TERMINATOR_SEQ
    """Set accelerometer's total acquisition time (all types of analysis)."""
    COMMAND_OVL = 'OVL' + TERMINATOR_SEQ
    """Set accelerometer's overlapping percentage between two consecutive FFT
    analysis."""


    # Requests from the masterboard.

    REQUEST_MOD = 'SELECT MASTER OR DEVICE' + TERMINATOR_SEQ
    REQUEST_IC = 'INSERT ADDRESS IC :' + TERMINATOR_SEQ
    REQUEST_OPM = 'INSERT OPERATING MODE :' + TERMINATOR_SEQ
    REQUEST_RADRS = 'INSERT REGISTER ADDRESS :' + TERMINATOR_SEQ
    REQUEST_RVAL = 'INSERT REGISTER VALUE :' + TERMINATOR_SEQ
    REQUEST_PARAMETER_NAME = 'INSERT PARAMETER NAME :'
    REQUEST_PARAMETER_VALUE = 'INSERT PARAMETER VALUE :'
    REQUEST_SLAVE = 'INSERT SLAVE NODE :'
    REQUEST_SENSOR_COMMAND = 'INSERT SENSOR COMMAND :'
    REQUEST_MEASURE_TYPE = 'INSERT MEASURE TYPE :'


    # Messages from the masterboard.

    MESSAGE_PROGRAMMING_DONE = 'PROGRAMMING DONE' + TERMINATOR_SEQ \
        + TERMINATOR_SEQ
    MESSAGE_CONNECTED = 'CONNECTED' + TERMINATOR_SEQ
    MESSAGE_TRANSMISSION_COMPLETED = 'TRANSMISSION COMPLETED' + TERMINATOR_SEQ
    MESSAGE_SENSOR_FAILED = 'SENSOR FAILED' + TERMINATOR_SEQ
    MESSAGE_PARAMETER_UPDATED = 'PARAMETER UPDATED' + TERMINATOR_SEQ
    MESSAGE_PARAMETER_ERROR = 'PARAMETER ERROR' + TERMINATOR_SEQ


# PARAMETER VALUES

class ODR(Enum):
    """Allowed parameters' values for ODR.
    Accelerometer's output data rate."""
    ODR_6660 = 6660
    ODR_3330 = 3330
    ODR_1660 = 1660
    ODR_833 = 833
    ODR_416 = 416
    ODR_208 = 208
    ODR_104 = 104
    ODR_52 = 52
    ODR_26 = 26
    ODR_13 = 13


class FLS(Enum):
    """Allowed parameters' values for FLS.
    Accelerometer's full scale."""
    FLS_2 = 2
    FLS_4 = 4
    FLS_8 = 8
    FLS_16 = 16


class SZE(Enum):
    """Allowed parameters' values for SZE.
    Accelerometer's input array size for FFT."""
    SZE_256 = 256
    SZE_512 = 512
    SZE_1024 = 1024
    SZE_2048 = 2048


class SUB(Enum):
    """Allowed parameters' values for SUB.
    Accelerometer's number of subranges."""
    SUB_8 = 8
    SUB_16 = 16
    SUB_32 = 32
    SUB_64 = 64


# Allowed parameters' values for ACQ. 
# Accelerometer's total acquisition time.
ACQ_MIN = 500
ACQ_MAX = 60000


# Allowed parameters' values for OVL.
# Accelerometer's overlapping percentage between two consecutive FFT analysis.
OVL_MIN = 5
OVL_MAX = 95
