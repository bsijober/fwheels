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
import sys
import numpy as np
from sensbiotk.transforms3d import quaternions as q
from sensbiotk.transforms3d import eulerangles as euler

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

class MainRun:
    def __init__(self):
        self.thread_imu = threading.Thread(name = "IMU ACQUI", target = imu.main)
        self.thread_read_files = threading.Thread(name = "READ FILES", target = self.read_files)
        self.thread_compute_values = threading.Thread(name = "COMPUTE VALUES", target = self.compute_values)         
         
#        self.imu_out = '/home/cybathlon/Bureau/Cybathlon/data_files/data_imu.txt'
        self.imu_out = open('/home/cybathlon/Bureau/Cybathlon/data_files/data_imu.txt','r')
        self.data_out = open('/home/cybathlon/Bureau/Cybathlon/data_files/data_out.txt','w+')        
        self.data_out.write('time \t pedal_angle \t bike speed \t cadence \n')
        
        self.quat_offset = [0, 0, 0]
        self.run = True
        
        self.screen = pygame.display.set_mode([500,500])
    
    def read_files(self):        
        while self.run:
            line_read = self.imu_out.readline()
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
                        self.bike_speed = (-self.gyr_z_wheel*0.325)*3.6
            except:
                continue
            
            
    def compute_values(self):
        while self.run:
            try:
                quat = euler.euler2quat(self.heading*np.pi/180,self.attitude*np.pi/180,self.bank*np.pi/180)
                if self.quat_offset == [0, 0, 0]:
                    self.quat_offset = quat
                    print('======')
                    print(str(self.quat_offset))
                else:
                    quat = q.mult(q.conjugate(self.quat_offset),quat)
                self.rad_pedal_angle = euler.quat2euler2(quat)[0]
                self.pedal_angle = (-self.rad_pedal_angle*180/np.pi)
                if self.pedal_angle < 0 :
                    self.pedal_angle = self.pedal_angle + 360
                self.plot()
                time.sleep(0.010)
                
                #save computed values in file
                self.data_out.write('%.1f \t %.1f \t %.1f \t %.1f \n' %(self.time_clock_imu1, self.pedal_angle, self.bike_speed, ((self.gyr_z*60)/(2*np.pi))))
            
            except:
                continue
            
    def plot(self):
        self.screen.fill([0,0,0])
        pygame.draw.arc(self.screen, (255,255,255), (100,100,300,300), 0, self.pedal_angle*np.pi/180, 15)
        pygame.draw.arc(self.screen, (255,0,0), (115,115,270,270), np.pi, self.pedal_angle*np.pi/180+np.pi, 15)        
        # Display some text
        font = pygame.font.Font(None, 36)
        text = font.render('%.1f deg  %.1f km/h  %.1f rpm' %(self.pedal_angle, self.bike_speed, ((self.gyr_z*60)/(2*np.pi))), 1, (255, 255, 255))
        textpos = text.get_rect()
        textpos.centerx = self.screen.get_rect().centerx
        self.screen.blit(text, textpos)
        pygame.display.update()
        
        
        
            
def main():
    #Init class
    m = MainRun()
    
    #Init Pygame
    pygame.init()
    pygame.display.set_caption('=== FREEWheels ===')
   
    # Start threads
    m.thread_imu.start()
    time.sleep(1)
    m.thread_read_files.start()
    time.sleep(4)
    m.thread_compute_values.start()
    
    while True:
        try:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_1:
                        print ('ONE')
                    if event.key == pygame.K_q:
                        logging.info('EXIT')
                        m.run = False
                        m.data_out.close()
                        m.imu_out.close()
                        pygame.quit()
                        return
            time.sleep(0.200)
        except KeyboardInterrupt:
            m.run = False
            pygame.quit()
            return
   
        
        
if __name__ == "__main__":
    main()