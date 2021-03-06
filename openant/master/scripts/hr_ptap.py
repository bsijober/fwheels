#Embedded file name: openant/master/scripts/hr_ptap.py
from __future__ import absolute_import, print_function
from ant.easy.node import Node
from ant.easy.channel import Channel
import logging
import threading
import array
import time
import sys
NETWORK_KEY = [185,
 165,
 33,
 251,
 189,
 114,
 195,
 69]

class HRPTAP:

    def __init__(self):
        self.init = True
        self.node = Node()
        self.node.set_network_key(0, NETWORK_KEY)
        self.channel_ptap = self.node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE)
        self.channel_ptap.on_broadcast_data = self.on_data_ptap
        self.channel_ptap.on_burst_data = self.on_data_ptap
        self.channel_ptap.set_period(8182)
        self.channel_ptap.set_search_timeout(30)
        self.channel_ptap.set_rf_freq(57)
        self.channel_ptap.set_id(32848, 11, 5)
        self.update_event_count = 0
        self.power = 0
        self.total_power = 0
        self.init_power = 0
        self.channel_hr = self.node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE)
        self.channel_hr.on_broadcast_data = self.on_data_hr
        self.channel_hr.on_burst_data = self.on_data_hr
        self.channel_hr.set_period(8070)
        self.channel_hr.set_search_timeout(12)
        self.channel_hr.set_rf_freq(57)
        self.channel_hr.set_id(0, 120, 0)
        self.hr = 0

    def on_data_hr(self, data):
        self.hr = data[7]

    def on_data_ptap(self, data):
        page_number = format(data[0], '02X')
        if page_number == '10':
            if data[3] != 255 and data[1] != self.update_event_count:
                cadence = data[3]
                self.power = int(data[7] << 8 | data[6])
                if self.init == True:
                    self.init_power = int(data[5] << 8 | data[4])
                    self.init = False
                self.total_power = int(data[5] << 8 | data[4]) - self.init_power
                self.update_event_count = data[1]
        elif page_number == '13':
            if data[3] != 255 and data[2] != 255 and data[1] != self.update_event_count:
                right_torque = data[3] * 0.5
                left_torque = data[2] * 0.5
                self.update_event_count = data[1]
                print('Right Torque : ' + str(right_torque) + '% || Left Torque : ' + str(left_torque) + '%')
        elif page_number == '01':
            if data[1] == 172:
                print('Calibration Successful')
            elif data[1] == 175:
                print('Calibration Failed')
            if data[2] == 0:
                print('Autozero is OFF')
            elif data[2] == 1:
                print('Autozero is ON')
            elif data[2] == 255:
                print('Autozero is NOT SUPPORTED')
        elif page_number == '02':
            if data[1] == 1:
                print('Crank length : ' + str(data[4] * 0.5 + 110))
                sensor_status = bin(data[5])[2:10]
                sensor_capabilities = bin(data[6])[2:10]
                if sensor_status[-2:] == '00':
                    print('Crank length invalid')
                elif sensor_status[-2:] == '10':
                    print('Default crank length used')
                elif sensor_status[-2:] == '01':
                    print('Crank length set manually')
                elif sensor_status[-2:] == '11':
                    print('Crank length automatically set')
                print('Firmware status : ' + str(sensor_status[-4:-2]))
                print('Sensor avail. :' + str(sensor_status[-6:-4]))
                if sensor_status[0:2] == '00':
                    print('Custom calibration NA')
                if sensor_capabilities == '0':
                    print('Auto crank length NA')
        elif page_number == '52':
            print('Battery identifier:' + str(bin(data[2])[2:10]))
            print('Battery capacity(%) : ' + str(data[6] / 256.0 * 100))

    def start_node(self):
        self.node.start()

    def stop_node(self):
        self.node.stop()
        self.text.insert('1.0', 'NODE STOPPED')

    def auto_zero(self):
        self.text.insert('1.0', 'AUTO ZERO REQUEST')
        auto_zero_req = array.array('B', [1,
         171,
         1,
         255,
         255,
         255,
         255,
         255])
        self.channel.send_acknowledged_data(auto_zero_req)

    def manual_zero(self):
        self.text.insert('1.0', 'MANUAL ZERO REQUEST')
        man_zero_req = array.array('B', [1,
         170,
         255,
         255,
         255,
         255,
         255,
         255])
        self.channel.send_acknowledged_data(man_zero_req)

    def request(self):
        self.text.insert('1.0', 'REQUEST')
        self.channel.send_acknowledged_data(array.array('B', [70,
         255,
         255,
         1,
         0,
         0,
         2,
         1]))


def main():
    logging.basicConfig()
    powertap = Powertap()
    thread_node = threading.Thread(None, target=powertap.start_node)
    try:
        powertap.channel.open()
        thread_node.start()
        print('thread started')
    except:
        print('error')


if __name__ == '__main__':
    main()
