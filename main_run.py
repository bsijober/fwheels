# -*- coding: utf-8 -*-
"""
Created on Fri Jan 29 11:33:19 2016

@author: cybathlon
"""

#import openant.master.scripts.hr_ptap as hr_ptap
import imu.scripts.imu_dump_full as imu
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
        self.thread_imu = threading.Thread(name = "IMU ACQUI", target = imu.main)
        self.thread_read_files = threading.Thread(name = "READ FILES", target = self.read_files)
        self.thread_compute_values = threading.Thread(name = "COMPUTE VALUES", target = self.compute_values)              
        self.thread_plot = threading.Thread(name = 'PLOT', target = self.thread_update_plot)
        self.file_number = 1        
        self.rec = False
        self.imu_out = open('/home/cybathlon/Bureau/Cybathlon/data_files/data_imu.txt','r')
        self.data_out = open('/home/cybathlon/Bureau/Cybathlon/data_files/data_out_'+str(self.file_number)+'.txt','w+')        
        self.data_out.write('time \t pedal_angle(deg) \t bike speed(km/h) \t distance(m) \t cadence(rpm) \t power(W) \t total_power(W) \t HR(bpm) \n')
        
        self.quat_offset = [0, 0, 0]
        self.offset_time = time.time()
        self.distance = 0
        self.init = True
        self.run = True
        self.old_line = 0
        self.flag_1 = False
        self.flag_2 = False
         #Init ANT devices 
#        self.hr_ptap = hr_ptap.HRPTAP()
#        thread_node = threading.Thread(None, target = self.hr_ptap.start_node)
#        self.hr_ptap.channel_hr.open()
#        self.hr_ptap.channel_ptap.open()
#        thread_node.start()
        
        self.screen = pygame.display.set_mode([800,500])
        
    def reset(self):
        self.rec = True
        self.offset_time = time.time()
        self.file_number = self.file_number + 1
        self.distance = 0
#        self.hr_ptap.init = True
        self.data_out = open('/home/cybathlon/Bureau/Cybathlon/data_files/data_out_'+str(self.file_number)+'.txt','w+')        
        self.data_out.write('time \t pedal_angle(deg) \t bike speed(km/h) \t distance(m) \t cadence(rpm) \t power(W) \t total_power(W) \t HR(bpm) \n')
        print('RESETTED, file number :'+str(self.file_number))
        
    def read_files(self):          
        while self.run:     
            try:
                line_read = self.imu_out.readline()
                if line_read != self.old_line:
                    line = [float(x) for x in line_read.split()]
                    if len(line) == 16:
                        src = line[0]
                        if src == 1: #IMU 1
                            [self.time_clock_imu1, self.t_full, self.bank, self.attitude, self.heading, self.t_raw, \
                            self.acc_x, self.acc_y, self.acc_z, self.mag_x, self.mag_y, self.mag_z, self.gyr_x, self.gyr_y, self.gyr_z] \
                            = line[1:16]
                            self.flag_1 = True
    #                        print('bank '+str(self.bank)+'attitude '+str(self.attitude) +'heading '+str(self.heading))
                        elif src == 2: #IMU2
                            self.time_clock_imu2 = time.time()                        
                            if self.init == True:
                                self.time_prev_imu2 = time.time()
                                self.init = False
                            self.delta_time = self.time_clock_imu2 - self.time_prev_imu2
                            self.time_prev_imu2 = self.time_clock_imu2    
                            self.gyr_z_wheel = line[15]
                            self.bike_speed = (-self.gyr_z_wheel*0.22)*3.6 #0.325 pour la roue en 26
                            self.distance = self.distance + (self.delta_time/3.6)*self.bike_speed
                            self.flag_2 = True
                    self.old_line = line_read
                    if self.flag_1 and self.flag_2 and time.time()-self.offset_time >4:  
                        self.compute_values()
                        self.flag_1 = False
                        self.flag_2 = False
               

            except:
                continue
            
            
    def compute_values(self):
#        while self.run:
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
#                self.plot()
                
                #save computed values in file
                if self.rec == True:
#                    self.data_out.write('%.1f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \n' %(self.time_clock_imu1 - self.offset_time, self.pedal_angle, self.bike_speed, self.distance, ((self.gyr_z*60)/(2*np.pi)), self.hr_ptap.power, self.hr_ptap.total_power, self.hr_ptap.hr))          
                    self.data_out.write('%.3f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \t %.1f \n' %(time.time() - self.offset_time, self.pedal_angle, self.bike_speed, self.distance, ((self.gyr_z*60)/(2*np.pi)), 0, 0, 0))
#                while not self.new_line:
#                     time.sleep(0.001)
               
            except Exception as e: 
                print(e)
#                continue
            
    def thread_update_plot(self):
        while self.run:
            self.plot()
            time.sleep(0.1)
            
    def plot(self):
        self.screen.fill([0,0,0])
        pygame.draw.arc(self.screen, (255,255,255), (100,100,300,300), 0, self.pedal_angle*np.pi/180, 15)
        pygame.draw.arc(self.screen, (255,0,0), (115,115,270,270), np.pi, self.pedal_angle*np.pi/180+np.pi, 15)        
#        power = self.hr_ptap.power
#        hr = self.hr_ptap.hr
#        total_power = self.hr_ptap.total_power
        power = 0
        hr = 0
        total_power = 0
#        pygame.draw.rect(self.screen, (255-power*10, power*10,0), (500,400,180, -power*10))        
#        pygame.draw.rect(self.screen, (255, 255, 255), (500,400,180, -300), 1)        
        # Display circle if record ON
        if self.rec == True:    
            pygame.draw.circle(self.screen, (255, 0, 0), (50,50), 10)         
        
        # Display some text
        font = pygame.font.Font(None, 20)
        text_pw = font.render('-- 30W', 1, (255, 255, 255))
        textpos_pw = text_pw.get_rect()
        textpos_pw.centerx = 700
        textpos_pw.centery = 98
        text_pw15 = font.render('-- 15W', 1, (255, 255, 255))
        textpos_pw15 = text_pw.get_rect()
        textpos_pw15.centerx = 700
        textpos_pw15.centery = 248
        text_pw0 = font.render('-- 0W', 1, (255, 255, 255))
        textpos_pw0 = text_pw.get_rect()
        textpos_pw0.centerx = 700
        textpos_pw0.centery = 398
        font = pygame.font.Font(None, 38)
        text = font.render('%.1f deg  %.1f rpm %.1f km/h  %.1f m %.1f s' %(self.pedal_angle, ((self.gyr_z*60)/(2*np.pi)), self.bike_speed, self.distance, time.time() - self.offset_time), 1, (255, 255, 255))
        textpos = text.get_rect()
        textpos.centerx = self.screen.get_rect().centerx
        text2 = font.render('inst: %.1f W  total: %.1f W  %.1f bpm' %(power, total_power, hr), 1, (255, 255, 255))
        textpos2 = text2.get_rect()
        textpos2.centerx = self.screen.get_rect().centerx
        textpos2.centery = 450
        self.screen.blit(text, textpos)
        self.screen.blit(text2, textpos2)
        self.screen.blit(text_pw, textpos_pw)
        self.screen.blit(text_pw0, textpos_pw0)
        self.screen.blit(text_pw15, textpos_pw15)
#        self.screen.blit(text, textpos)
        pygame.display.update()
       
        
        
            
def main():
    logging.disable('DEBUG')

    #Init class
    m = MainRun()
    
    #Init Pygame
    pygame.init()
    pygame.display.set_caption('=== FREEWheels ===')
    
    # Start threadsq
    m.thread_imu.start()
    time.sleep(0.001)
#    m.thread_ptap.start()
#    time.sleep(1)
    m.thread_read_files.start()
    time.sleep(5)
#    m.thread_compute_values.start()
#    time.sleep(1)
    m.thread_plot.start()
#    m.thread_hr.start()
#    time.sleep(0.2)
   
    
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
                        logging.info('EXIT')
                        m.run = False
                        m.data_out.close()
                        m.imu_out.close()
                        pygame.quit()
                        return
            time.sleep(0.400)
        except KeyboardInterrupt:
            m.run = False
            m.data_out.close()
            m.imu_out.close()
            pygame.quit()
            return
   
        
        
if __name__ == "__main__":
    main()