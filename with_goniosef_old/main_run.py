# -*- coding: utf-8 -*-
"""
Created on Fri Jan 29 11:33:19 2016

@author: cybathlon
"""

import imu.scripts.imu_dump_full_mod as imu
import threading
import logging
import time
import pygame
import numpy as np
from sensbiotk.transforms3d import quaternions as q
from sensbiotk.transforms3d import eulerangles as euler

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

class MainRun:
    def __init__(self):
      
        self.file_number = 0        
        self.rec = False
        self.imu_out = open('./data_files/data_imu.txt','r')
        self.data_out = open('./data_files/data_out_'+str(self.file_number)+'.txt','w+')        
        self.data_out.write('time \t pedal_angle(deg) \t bike speed(km/h) \t distance(m) \t cadence(rpm)\n')
        
        self.quat_offset = [0, 0, 0]
        self.offset_time = time.time()
        self.distance = 0
        self.init = True
        
        self.screen = pygame.display.set_mode([800,500])
        
    def reset(self):
        self.rec = True
        self.offset_time = time.time()
        self.file_number = self.file_number + 1
        self.distance = 0
        self.data_out = open('./data_files/data_out_'+str(self.file_number)+'.txt','w+')        
        self.data_out.write('time \t pedal_angle(deg) \t bike speed(km/h) \t distance(m) \t cadence(rpm)\n')
        print('RESETTED, file number :'+str(self.file_number))

    def thread_process_data(self, e):
        while not e.is_set():
            try:
                self.imu_out = open('./data_files/data_imu.txt','r')
                line_1 = self.imu_out.readline()
                line_2 = self.imu_out.readline()  
                line_1 = [float(x) for x in line_1.split()]
                line_2 = [float(x) for x in line_2.split()]               
                
                #IMU 1
                [src, tfull, self.bank, self.attitude, self.heading, \
                self.acc_x, self.acc_y, self.acc_z, self.gyr_x, self.gyr_y, self.gyr_z] \
                = line_1[0:12]
                                      
                # IMU 2
                self.time_clock_imu2 = time.time()                        
                if self.init == True:
                    self.time_prev_imu2 = time.time()
                    self.init = False
                self.delta_time = self.time_clock_imu2 - self.time_prev_imu2
                self.time_prev_imu2 = self.time_clock_imu2    
                self.gyr_z_wheel = line_2[10]
                self.bike_speed = (-self.gyr_z_wheel*0.22)*3.6 #0.325 pour la roue en 26
                self.distance = self.distance + (self.delta_time/3.6)*self.bike_speed
                        
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
                #save computed values in file
                if self.rec == True:
                   self.data_out.write('%.3f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \n' %(time.time() - self.offset_time, self.pedal_angle, self.bike_speed, self.distance, ((self.gyr_z*60)/(2*np.pi)), 0, 0, 0))
                time.sleep(0.001)
            except Exception as error: 
                print(error)    
            
    def thread_plt(self, e):
        while not e.is_set():
            try:
                self.screen.fill([0,0,0])
                pygame.draw.arc(self.screen, (255,255,255), (100,100,300,300), 0, self.pedal_angle*np.pi/180, 15)
                pygame.draw.arc(self.screen, (255,0,0), (115,115,270,270), np.pi, self.pedal_angle*np.pi/180+np.pi, 15)        
           
                # Display circle if record ON
                if self.rec == True:    
                    pygame.draw.circle(self.screen, (255, 0, 0), (50,50), 10)         
                
                # Display some text
                font = pygame.font.Font(None, 38)
                text = font.render('%.1f deg  %.1f rpm %.1f km/h  %.1f m %.1f s' %(self.pedal_angle, ((self.gyr_z*60)/(2*np.pi)), self.bike_speed, self.distance, time.time() - self.offset_time), 1, (255, 255, 255))
                textpos = text.get_rect()
                textpos.centerx = self.screen.get_rect().centerx
               
                self.screen.blit(text, textpos)
                pygame.display.update()
                time.sleep(0.1)
            except Exception as error: 
                print(error)            
        
            
def main():
    logging.disable('DEBUG')

    #Init class
    m = MainRun() 
    
    # Variables useful
    e = threading.Event()
    
    # Threads
    # Init IMUs
    thread_imu = threading.Thread(name = "IMU ACQUI", target = imu.main)
    thread_process_data = threading.Thread(name ="Process Data", target = m.thread_process_data, args = (e,))
    thread_plt = threading.Thread(name='Plt', target = m.thread_plt, args = (e,))
    #Init Pygame
    pygame.init()
    pygame.display.set_caption('=== FREEWheels ===')
    
    # Start threadsq
    thread_imu.start()
    time.sleep(4)
    thread_process_data.start()
    time.sleep(1)
    thread_plt.start()
    
    while True:
        try:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        print ('R PRESSED FOR RESET')
                        m.reset()
                    if event.key == pygame.K_s:
                        print ('S PRESSED FOR START/STOP RECORDING')
                        m.rec = not m.rec
                    if event.key == pygame.K_q:
                        pygame.quit()
                        logging.info('EXIT')
                        e.set()
                        m.data_out.close()
                        m.imu_out.close()
                        return
            time.sleep(0.400)
        except KeyboardInterrupt:
            e.set()
            m.data_out.close()
            m.imu_out.close()
            pygame.quit()
            return
   
        
        
if __name__ == "__main__":
    main()