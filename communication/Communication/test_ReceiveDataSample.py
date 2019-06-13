from digi.xbee.devices import XBeeDevice
import json


# Copyright 2017, Digi International Inc.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''Esto es para el bote, el bote envia a la estacion cada 500ms'''
#****************************************************************************************#
# Replace with the serial port where your local module is connected to.
PORT = "/dev/ttyUSB1"
# Replace with the baud rate of your local module.
BAUD_RATE = 9600
#REMOTE_NODE_ID = "vtecstation" #El nodo con el que se quiere comunicar.
REMOTE_NODE_ID = "vtecboat"
#****************************************************************************************#


def main():

    print(" +-------------------------------------------------+")
    print(" |                       Bote                      |")
    print(" +-------------------------------------------------+\n")
    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()
        device.flush_queues()

        print("Waiting conversation...\n")
        
        #Variable to stop the conversation 
        commActive = True

        while commActive :

            #Read data and chek if something has been received 
            xbee_message = device.read_data()
            if xbee_message is not None:
                
                #Print the message and 
                print("Received Message: " , xbee_message.data.decode())
                print(xbee_message.data.decode())

                #if it's different than exit continue listening
                if xbee_message.data.decode() == 'exit':
                    commActive = False

    #If the device is not closed, close it.
    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()