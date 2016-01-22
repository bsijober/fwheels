# ANT - Hearth Rate Monitor Example
#
# Copyright (c) 2012, Gustav Tiger <gustav@tiger.name>
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

from __future__ import absolute_import, print_function

from ant.easy.node import Node
from ant.easy.channel import Channel
from ant.base.message import Message

import logging
import struct
import threading
import sys
import Tkinter as tk
import numpy as np
import array


NETWORK_KEY= [0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45]

class Powertap:
    def __init__(self):
        
        self.node = Node()
        self.node.set_network_key(0x00, NETWORK_KEY)
        
        self.channel = self.node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE)
    
        self.channel.on_broadcast_data = self.on_data
        self.channel.on_burst_data = self.on_data
        self.channel.set_period(8182)
        self.channel.set_search_timeout(30)
        self.channel.set_rf_freq(57)
        self.channel.set_id(32848,11,5)
        
        self.update_event_count = 0
    
    #### DISPLAY ####    
        self.root = tk.Tk()
        self.root.geometry('300x200')
        self.text = tk.Text(self.root, background='black', foreground='white', font=('Comic Sans MS', 12))
        self.text.pack()
        self.root.bind('<KeyPress>', self.onKeyPress)
    
    def on_data(self, data):
        
        page_number = format(data[0], '02X')
        print('Page number : '+ page_number)

        if page_number == '10': #simple power message
            if data[3] != 255 and data[1] != self.update_event_count:
                cadence = data[3]
                power = int( (data[7]<<8) | data[6]) #convert LSB and MSB to int
                self.update_event_count = data[1]
                print('Cadence : ' + str(cadence) +' || Power : ' + str(power) +'W' +' ('+str(hex(data[6])) +'|' + str(hex(data[7])) +')')
#                self.text.insert('1.0', 'Cadence:  %s\n' % (cadence, ))
#                print(str(data[6
#                self.text.see('1.0')
        
        
        elif page_number == '01': #calibration response main data page
            if data[1] == 0xAC:
                print('Calibration Successful')
            elif data[1] == 0xAF:
                print('Calibration Failed')
            if data[2] == 0x00:
                print('Autozero is OFF')
            elif data[2] == 0x01:
                print('Autozero is ON')
            elif data[2] == 0xFF:
                print('Autozero is NOT SUPPORTED')
                
        elif page_number =='02': #bike parameters
            if data[1] == 0x01: #crank parameters
                print('Crank length : '+str((data[4]*0.5+110)))
                sensor_status = bin(data[5])[2:10]
                sensor_capabilities = bin(data[6])[2:10]
                if sensor_status[-2:] == '00': #crank length status
                    print('Crank length invalid')
                elif sensor_status[-2:] == '10':
                    print('Default crank length used')
                elif sensor_status[-2:] == '01':
                    print('Crank length set manually')
                elif sensor_status[-2:] == '11':
                    print('Crank length automatically set')
                
                print('Firmware status : '+ str(sensor_status[-4:-2])) # firmware status
                print('Sensor avail. :'+ str(sensor_status[-6:-4])) #s sensor avilability
                if sensor_status[0:2] == '00': print('Custom calibration NA')
                if sensor_capabilities == '0': print('Auto crank length NA')
                    
        elif page_number == '52': 
            print('Battery identifier:' +str(bin(data[2])[2:10]))
            print('Battery capacity(%) : ' + str((data[6]/256.0)*100))
                
                
    def start_node(self):
         self.node.start()
         
    def stop_node(self):
        self.node.stop()
        self.text.insert('1.0', 'NODE STOPPED')
         
    def start_graph(self):
        self.root.mainloop()
        
    def auto_zero(self):
        self.text.insert('1.0', 'AUTO ZERO REQUEST')
        auto_zero_req = array.array('B', [0x01, 0xab, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff])
        self.channel.send_acknowledged_data(auto_zero_req)   
        
    def manual_zero(self):
        self.text.insert('1.0', 'MANUAL ZERO REQUEST')
        man_zero_req = array.array('B', [0x01, 0xaa, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff])
        self.channel.send_acknowledged_data(man_zero_req)
        
    def request(self):
        self.text.insert('1.0', 'REQUEST')
#        req = array.array('B', [0x46, 0xff, 0xff, 0x, 0x00, 0xff, 0xff, 0xff])
        self.channel.send_acknowledged_data(array.array('B',[0x46, 0xff, 0xff, 0x01, 0x00, 0x00, 0x02, 0x01]))
#        self.channel.request_message()
        
    def onKeyPress(self, event):
        self.text.insert('1.0', '\n You pressed %s\n' % (event.char, ))
        if event.char =='z':
            self.manual_zero()
        elif event.char =='a':
            self.auto_zero()
        elif event.char =='r':
            self.request()
        elif event.char =='q':
            self.stop_node()
            

def main():
    logging.basicConfig()

    powertap = Powertap()    
    thread_node = threading.Thread(None, target = powertap.start_node)
    
    try:
        powertap.channel.open()
        thread_node.start()
        print('thread started')
        powertap.start_graph()
    except:
        print('error')
    
    
    
if __name__ == "__main__":
    main()

