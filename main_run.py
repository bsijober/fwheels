# -*- coding: utf-8 -*-
"""
Created on Fri Jan 29 11:33:19 2016

@author: cybathlon
"""


import imu.scripts.imu_dump_full as imu
import threading
import logging
import time
import pygame
import os
from sensbiotk.transforms3d import quaternions as q
from sensbiotk.transforms3d import eulerangles as euler

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

class MainRun:
    def __init__(self):
        self.thread_imu = threading.Thread(name = "IMU ACQUI", target = imu.main)
        self.thread_read_files = threading.Thread(name = "READ FILES", target = self.read_files)
        self.thread_compute_values = threading.Thread(name = "COMPUTE VALUES", target = self.compute_values)         
         
        self.imu_out = open('/home/cybathlon/Bureau/Cybathlon/data_files/data_imu.txt', 'r')
        self.quat_offset = [0, 0, 0]
    
    def read_files(self):        
        while True:
            line_read = self.imu_out.readline(self.imu_out.seek(os.SEEK_END))
            try:
                line = [float(x) for x in line_read.split()]
                if len(line) == 16:
                    src = line[0]
                    if src == 1: #IMU 1
                        [self.time_clock_imu1, self.t_full, self.bank, self.attitude, self.heading, self.t_raw, \
                        self.acc_x, self.acc_y, self.acc_z, self.mag_x, self.mag_y, self.mag_z, self.gyr_x, self.gyr_y, self.gyr_z] \
                        = line[1:16] 
#                        print('bank '+str(self.bank)+'attitude '+str(self.attitude) +'heading '+str(self.heading))
                    elif src == 2: #IMU2
                        self.time_clock_imu2 = line[1]
                        self.gyr_z_wheel = line[15]
                time.sleep(0.010)
            except:
                continue
            
            
    def compute_values(self):
        while True:
            try:
                quat = euler.euler2quat(self.heading*np.pi/180,self.attitude*np.pi/180,self.bank*np.pi/180)
                if self.quat_offset == [0, 0, 0]:
                    quat_offset = quat
                else:
                    quat = q.mult(q.conjugate(quat_offset),quat)
                z_angle = euler.quat2euler(quat)[0]
                print(str(z_angle))
                time.sleep(0.010)
            except:
                continue
            
            
            
def main():
    #Init class
    m = MainRun()
    
    #Init Pygame
    pygame.init()
    pygame.display.set_mode([300,300])
    pygame.display.set_caption('=== FREEWheels ===')
   
    # Start threads
    m.thread_imu.start()
    time.sleep(4)
    m.thread_read_files.start()
    m.thread_compute_values.start()
    
    while True:
        try:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_1:
                        print ('ONE')
                    if event.key == pygame.K_q:
                        logging.info('EXIT')
                        pygame.quit()
                        return
            time.sleep(0.200)
        except KeyboardInterrupt:
            pygame.quit()
            return
   
        
        
if __name__ == "__main__":
    main()