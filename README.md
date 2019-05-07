# WireST SDK

WireST SDK is a library for Linux gateways that allows easy access to the data exported by an IO-Link device that implements the [IOLinkST Protocol](https://github.com/STMicroelectronics/WireSTSDK_Python#IOLinkST-protocol).


## Documentation
Documentation can be found [here](https://stmicroelectronics.github.io/WireSTSDK_Python/index.html).


## Compatibility
This version of the SDK is compatible with [Python](https://www.python.org/) 2.7 and runs on a Linux system.


## Preconditions
The WireST SDK makes use of the [pySerial](https://github.com/pyserial/pyserial) Python interface to serial ports on Linux.
  ```Shell
  $ sudo pip install pyserial
  ```
Moreover, it uses the [concurrent.futures](https://docs.python.org/3/library/concurrent.futures.html) module to run pools of threads in background, that serve listeners' callbacks.
  ```Shell
  $ sudo pip install futures
  ```


## Installation
The WireST SDK can be installed through the Python pip package manager.
  ```Shell
  $ sudo pip install wire-st-sdk
  ```


## Setting up the application examples
Before running the application examples, please prepare your devices as described here below:
 * The [example_iolink_1.py](https://github.com/STMicroelectronics/WireSTSDK_Python/blob/master/wire_st_examples/iolink/example_iolink_1.py) application example shows how to connect IO-Link devices to a Linux gateway, to set their parameters, and to get data from them. The application requires to have a masterboard, e.g. the STEVAL-IPD004V1 evaluation board, connected to IO-Link devices, e.g. the STEVAL-IPD005V1 evaluation board, with a FW compatible with the [IOLinkST Protocol](https://github.com/STMicroelectronics/WireSTSDK_Python#IOLinkST-protocol).

Other application examples which show how to send data to the cloud can be found within the [EdgeST SDK](https://github.com/STMicroelectronics/EdgeSTSDK_Python) for Linux, an IoT edge computing abstraction library for Linux gateways.


## Running the application examples
To run the application examples please follow the steps below:
 1. Install the WireST SDK as described by the [Installation](https://github.com/STMicroelectronics/WireSTSDK_Python#installation) chapter.
 2. Clone the WireST SDK git repository to download the application examples:
    ```Shell
    $ git clone https://github.com/STMicroelectronics/WireSTSDK_Python.git
    ```
 3. Enter the "wire_st_examples/iolink" folder and run the desired script:
    ```Shell
    $ sudo python example_iolink_x.py
    ```


## License
COPYRIGHT(c) 2019 STMicroelectronics

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above 
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. Neither the name of STMicroelectronics nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
