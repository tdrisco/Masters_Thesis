#!/usr/bin/env python

"""
Author: Tim Driscoll
Date: 2/22/23

Description: All inclusive MTI Data Collection and processing script
            1. Collect raw euler angle readings Xsens MTI
            2. Realtime filtering of euler angle readings
            3. Realtime calculation of phase variable
            4. Actively saving data in CSV file
            5. Vicon interrupt triggered start and stop
            6. Realtime data visualization made possible with kst
"""

import MTI_Setup
import MID_Codes

import RPi.GPIO as GPIO

import time
import datetime

import csv

import numpy as np
from math import sqrt, cos, sin, atan2, pi

_RUNTIME = 10

_RASPBERRYPI = True

_CSVFILENAME = "kst.csv"

vicon = 26
motionCap_flag = False
capFinished = False

GPIO.setmode(GPIO.BCM)
GPIO.setup(vicon, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def vicon_end(vicon):
    global motionCap_flag
    global capFinished
    if motionCap_flag == False:
        motionCap_flag = True
        #print("Motion Capture Started-----------------")
    else:
        motionCap_flag = False
        #print("Motion Capture Ended--------------------")
        #capFinshed = True

GPIO.add_event_detect(vicon, GPIO.BOTH, callback=vicon_end)

#Setup Data collction variables----------------------------

#slow: 0.67 m/s - 1.5 mph
#normal: 1.00 m/s - 2.25 mph
#fast: 1.35 m/s - 3.00 mph

#acceleration: 0.1 m/s 

subject = ["AB01","AB02","AB03","AB04"]
treadmillSpeed = ["slow","normal","fast"]
treadmillIncline = ["neg10","neg7.5","neg5","neg2.5","0","2.5","5","7.5","10"]
folder = "5_16_Test1_AB01"

#Setup Data collection variables for second set of test----
subject2 = ["AB01","AB02","AB03","AB04"]
treadmillSpeed2 = ["slow","normal","fast"]
treadmillIncline2 = ["neg10","neg7.5","neg5","neg2.5","0","2.5","5","7.5","10"]
folder2 = "5_16_Test2_AB01"

class XSensDriver(object):

    def __init__(self, Filter=True):

        #Tune these parameters depeding on Pi or computer
        if _RASPBERRYPI:
            device = '/dev/ttyUSB0'
            timeout = 0.001
        else:
            device = 'COM10'
            timeout = 0.002
        
        baudrate = 115200
        
        if device == 'auto':
            devs = MTI_Setup.find_devices()
            if devs:
                device, baudrate = devs[0]
                print("Detected MT device on port %s @ %d bps" % (device, baudrate))
            else:
                print("Fatal: could not find proper MT device.")
                print("Could not find proper MT device.")
                return
        if not baudrate:
            baudrate = MTI_Setup.find_baudrate(device)
        if not baudrate:
            print("Fatal: could not find proper baudrate.")
            print("Could not find proper baudrate.")
            return

        print("MT node interface: %s at %d bd." % (device, baudrate))

        # Enable config mode
        self.mt = MTI_Setup.MTDevice(device, baudrate, timeout, True, False)

        # Configure (see bottom of mtdevice.py)
        if Filter:
            self.EKF_Setup()
            self.FilterSwitch = True
        
        output_config = MTI_Setup.get_output_config('if2000,oe400fw,wr')
        print("Changing output configuration")
        self.mt.SetOutputConfiguration(output_config)
        print("System is Ok, Ready to Record.")
    
    def file_setup(self,filename=_CSVFILENAME):
        
        #Reset all the data collection variables
        self.count = 0

        self.delta_t = []
        self.yaw = []
        self.pitch = []
        self.roll = []
        self.angVel_x = []
        self.angVel_y = []
        self.angVel_z = []
        self.phaseVar = []
        self.EKFroll = []

        self.roll_cur = 0
        self.pitch_cur = 0
        self.angVelx_cur = 0
        self.angVely_cur = 0
        self.angVelz_cur = 0
        self.phaseVar_cur = 0
        self.EKFroll_cur = 0

        self.delta_t_curr = 0
    
        self.fpt = open(filename, "w", newline="")
        self.file = csv.writer(self.fpt,delimiter=",",quotechar="|",quoting=csv.QUOTE_MINIMAL)
        self.file.writerow(["Time [Sec]","Roll Angle [Deg]", "Pitch Angle [Deg]", "Phase Angle", "Angular Velocity X [deg/s]", "Angular Velocity Y [deg/s]", "Angular Velocity Z [deg/s]", "EKF Roll Angle [Deg]", "Step Frequency [Hz]", "Phase Variable"])
        
        print("Reload Data in KST now!")
        time.sleep(1.5)
        
    def EKF_Setup(self):
        self.estimates = []

        #Define the state variables and covariance 
        self.X = np.mat([[0], [0], [0]])
        self.S = np.mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.I = np.mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        self.Xt_t1 = np.mat([[0.0], [0.0], [0.0]])
        self.Kt = np.mat([[0], [0], [0]])
        self.St_t1 = np.mat([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

        #Define the Jacobians
        self.J_dfdx = np.mat([[1, 1, 0], [0, 1, 0], [(1/10)*cos(self.X[0,0]/10), 0, 0]])
        self.J_dfda = np.mat([[0, 0, 0], [0, 1, 0], [0, 0, 0]])
        self.J_dgdx = np.mat([0, 0, 1])
        self.J_dgdn = np.mat([1])

        #Define the noise covariance 
        # Dynamic noise covariance (how much prediction is trusted (0 is no noise))
        self.Q = np.mat([[0, 0, 0], [0, 0.001, 0], [0, 0, 0]])
        # Measurment Noise
        self.R = np.mat([1])
    
    def EKF_Run(self, InputVal):
        self.J_dfdx[2,0] = (1/10)*cos(self.X[0,0]/10) # Update The Jacobians

        self.Xt_t1[0,0] = self.X[0,0] + self.X[1,0]
        self.Xt_t1[1,0] = self.X[1,0]
        self.Xt_t1[2,0] = sin(self.X[0,0]/10)

        self.St_t1 = self.J_dfdx*self.S*self.J_dfdx.transpose() + self.J_dfda*self.Q*self.J_dfda.transpose()
        self.Yt = InputVal
        self.Kt = (self.St_t1*self.J_dgdx.getT()) * pow(self.J_dgdx*self.St_t1*self.J_dgdx.getT() + self.J_dgdn*self.R*self.J_dgdn.getT(), -1)

        self.g = sin(self.X[0,0]/10)
        self.X = self.Xt_t1 + self.Kt*(self.Yt - self.g)
        self.S = (self.I - self.Kt*self.J_dgdx)*self.St_t1

        #estimates.append(X[2,0])
        return self.X[2,0]

    def CDS_Setup(self):

        #Define the state variables and covariance 
        self.T = 1/100 #Sampling frequency
        self.M = 7 #Number of fourier series components
        self.eta = 1 #Learning Coefficient
        self.mu = 0.1 #Coupling Constant

        #Define intial frequency and phase variable
        self.w = [2*pi*2/5]
        self.phi = [0]

        self.ac = np.zeros(self.M)
        self.bc = np.zeros(self.M)

    def CDS_Run(self, InputVal):
        
        y = InputVal
        self.estimate = 0

        for c in range(self.M):
            self.estimate = self.estimate + self.ac[c] * cos(c*self.phi[-1]) + self.bc[c] * sin(c*self.phi[-1])
        
        self.error = y - self.estimate

        w_curr = self.w[-1]
        self.w.append(abs(w_curr - self.T*self.mu*self.error*sin(self.phi[-1])))

        for c in range(self.M):
            self.ac[c] = self.ac[c] + self.eta * self.T * cos(c*self.phi[-1])*self.error
            self.bc[c] = self.bc[c] + self.eta * self.T * sin(c*self.phi[-1])*self.error

        phi_curr = self.phi[-1]
        phi_next = (phi_curr + self.T*(w_curr - self.mu*self.error*sin(phi_curr))) % (2*pi)

        if(((phi_next - phi_curr) % (2*pi)) > (0.5*pi)):
            self.phi.append(phi_curr)
        else:
            self.phi.append(phi_next)

    def spin(self):
        try:
            #t_end = time.time() + _RUNTIME
            dummy = 0
            while not motionCap_flag:
                dummy = dummy + 1
            self.t_start = datetime.datetime.now()
            while motionCap_flag:#time.time() < t_end:
                # Spin to try to get new messages
                self.spin_once()
                self.count = self.count + 1
                #self.reset_vars()
            self.count = self.count - 1
            self.fpt.close()
        except KeyboardInterrupt:
            print("Data Stream Interrupted")
            pass

    def spin_once(self):
        '''Read data from device and publishes ROS messages.'''
        def convert_coords(x, y, z, source, dest='ENU'):
            """Convert the coordinates between ENU, NED, and NWU."""
            if source == dest:
                return x, y, z
            # convert to ENU
            if source == 'NED':
                x, y, z = y, x, -z
            elif source == 'NWU':
                x, y, z = -y, x, z
            # convert to desired
            if dest == 'NED':
                x, y, z = y, x, -z
            elif dest == 'NWU':
                x, y, z = y, -x, z
            return x, y, z

        def convert_quat(q, source, dest='ENU'):
            """Convert a quaternion between ENU, NED, and NWU."""
            def q_mult(q0, q1):
                """Quaternion multiplication."""
                (w0, x0, y0, z0) = q0
                (w1, x1, y1, z1) = q1
                w = w0*w1 - x0*x1 - y0*y1 - z0*z1
                x = w0*x1 + x0*w1 + y0*z1 - z0*y1
                y = w0*y1 - x0*z1 + y0*w1 + z0*x1
                z = w0*z1 + x0*y1 - y0*x1 + z0*w1
                return (w, x, y, z)
            q_enu_ned = (0, 1./sqrt(2), 1./sqrt(2), 0)
            q_enu_nwu = (1./sqrt(2), 0, 0, -1./sqrt(2))
            q_ned_nwu = (0, -1, 0, 0)
            q_ned_enu = (0, -1./sqrt(2), -1./sqrt(2), 0)
            q_nwu_enu = (1./sqrt(2), 0, 0, 1./sqrt(2))
            q_nwu_ned = (0, 1, 0, 0)
            if source == 'ENU':
                if dest == 'ENU':
                    return q
                elif dest == 'NED':
                    return q_mult(q_enu_ned, q)
                elif dest == 'NWU':
                    return q_mult(q_enu_nwu, q)
            elif source == 'NED':
                if dest == 'ENU':
                    return q_mult(q_ned_enu, q)
                elif dest == 'NED':
                    return q
                elif dest == 'NWU':
                    return q_mult(q_ned_nwu, q)
            elif source == 'NWU':
                if dest == 'ENU':
                    return q_mult(q_nwu_enu, q)
                elif dest == 'NED':
                    return q_mult(q_nwu_ned, q)
                elif dest == 'NWU':
                    return q

        def fill_from_Orient(orient_data):
            '''Fill messages with information from 'orientation' MTData block. '''
            self.pub_imu = True
            if 'quaternion' in orient_data:
                w, x, y, z = o
                print('orientation ='+str(orient_data['quaternion']))
            elif 'roll' in orient_data:
                print('orientation_data r='+str(orient_data['roll'])+',p='+str(orient_data['pitch'])+',y='+str(orient_data['yaw']))


        def fill_from_Auxiliary(aux_data):
            '''Fill messages with information from 'Auxiliary' MTData block.'''
            try:
                self.anin1_msg.data = o['Ain_1']
                self.pub_anin1 = True
            except KeyError:
                pass
            try:
                self.anin2_msg.data = o['Ain_2']
                self.pub_anin2 = True
            except KeyError:
                pass

        def fill_from_Pos(position_data):
            '''Fill messages with information from 'position' MTData block.'''
            print('Position x='+str(position_data['Lat'])+',y='+str(position_data['Lon'])+',z='+str(position_data['Alt']))

        def fill_from_Vel(velocity_data):
            '''Fill messages with information from 'velocity' MTData block.'''
            print('Velocity x='+str(velocity_data['Vel_X'])+',y='+str(velocity_data['Vel_Y'])+',z='+str(velocity_data['Vel_Z'])+',frame='+str(velocity_data['frame']))
            pass


        def fill_from_Stat(status):
            '''Fill messages with information from 'status' MTData block.'''
            if status & 0b0001:
                print("Status = Ok")
            else:
                 print("Status = Failed")
            if status & 0b0010:
                 print("Status = Valid")
            else:
                 print("Status = Invalid")
            pass

        def fill_from_Sample(ts):
            '''Catch 'Sample' MTData blocks.'''
            self.h.seq = ts

        def fill_from_Timestamp(o):
            '''Fill messages with information from 'Timestamp' MTData2 block.'''
            print(datetime.datetime.now(), end=" ")

        def fill_from_Orientation_Data(o):
            '''Fill messages with information from 'Orientation Data' MTData2
            block.'''
            
            try:
                x, y, z, w = o['Q1'], o['Q2'], o['Q3'], o['Q0']
                print('orientation_data x='+str(x)+',y='+str(y)+',z='+str(z)+',w='+str(w))
            except KeyError:
                pass
            try:
                print('Euler Angles - Roll: '+str(o['Roll'])+', Pitch: '+str(o['Pitch'])+',y='+str(o['Yaw']), end=" ")
                if self.count == 1:
                    self.t_start = datetime.datetime.now()
                    self.delta_t.append(0)
                else:
                    delta = datetime.datetime.now()-self.t_start
                    self.delta_t.append(delta.total_seconds() * 1000)
    
                self.roll.append(o['Roll'])
                self.pitch.append(o['Pitch'])
                self.yaw.append(o['Yaw'])
                self.roll_cur = o['Roll']
                self.pitch_cur = o['Pitch']
                self.delta_t_curr = self.delta_t[-1]
            except KeyError:
                pass

        def fill_from_Acceleration(o):
            '''Fill messages with information from 'Acceleration' MTData2 block.'''
            fill_from_Timestamp(o)
            print('acceleration_data x='+str(o['accX'])+',y='+str(o['accY'])+',z='+str(o['accZ']))
            pass

        def fill_from_Angular_Velocity(o):
            '''Fill messages with information from 'Angular Velocity' MTData2 block.'''
            if self.count != 0:
                
                print(' Angular Velocity x='+str(o['gyrX'])+',y='+str(o['gyrY'])+',z='+str(o['gyrZ']))
                #print(' Angular Velocity x='+str(o['gyrX']))
                self.angVel_x.append(o['gyrX'])
                self.angVelx_cur = o['gyrX']
                self.angVel_y.append(o['gyrY'])
                self.angVely_cur = o['gyrY']
                self.angVel_z.append(o['gyrZ'])
                self.angVelz_cur = o['gyrZ']
                pass

        def fill_from_Analog_In(o):
            '''Fill messages with information from 'Analog In' MTData2 block.'''
            try:
                self.anin1_msg.data = o['analogIn1']
                self.pub_anin1 = True
            except KeyError:
                pass
            try:
                self.anin2_msg.data = o['analogIn2']
                self.pub_anin2 = True
            except KeyError:
                pass

        def fill_from_Magnetic(o):
            '''Fill messages with information from 'Magnetic' MTData2 block.'''
            print('magnetic x='+str(o['magX'])+',y='+str(o['magY'])+',z='+str(o['magZ'])+',frame='+str(o['frame']))
            pass

        def fill_from_Velocity(o):
            '''Fill messages with information from 'Velocity' MTData2 block.'''
            print('velocity x='+str(o['velX'])+',y='+str(o['velY'])+',z='+str(o['velZ'])+',frame='+str(o['frame']))
            pass

        def fill_from_Status(o):
            '''Fill messages with information from 'Status' MTData2 block.'''
            try:
                status = o['StatusByte']
                fill_from_Stat(status)
            except KeyError:
                pass
            try:
                status = o['StatusWord']
                fill_from_Stat(status)
            except KeyError:
                pass

        def find_handler_name(name):
            return "fill_from_%s" % (name.replace(" ", "_"))

        # get data
        try:
            data = self.mt.read_measurement()
        except MID_Codes.MTTimeoutException:
            time.sleep(0.01)
            return
        # common header
        #self.h = Header()
        #self.h.stamp = rospy.Time.now()
        #self.h.frame_id = self.frame_id

        # set default values
        #self.reset_vars()

        # fill messages based on available data fields
        # publish available information
        # TODO: Actually do things here
        for n, o in data.items():
            try:
                locals()[find_handler_name(n)](o)
            except KeyError:
                print("Unknown MTi data packet: '%s', ignoring." % n)
        
        #bottom of spin function Calculate the phase variable
        #(Make this into its own function)
        if(self.count != 0):
            self.phaseVar_cur = atan2(-self.angVelz_cur,self.roll_cur)
            self.phaseVar.append(self.phaseVar_cur)
            if self.FilterSwitch:
                self.EKFroll_cur = self.EKF_Run(self.roll_cur)
                self.EKFroll.append(self.EKFroll_cur)
            self.CDS_Run(self.angVelz_cur * 180/pi)
            #After each data read write all the current values to kst csv
            self.file.writerow([self.delta_t_curr/1000, self.roll_cur, self.pitch_cur, self.phaseVar_cur, self.angVelx_cur * 180/pi, self.angVely_cur * 180/pi, self.angVelz_cur * 180/pi, self.EKFroll_cur, self.w[-1]/(2*pi), self.phi[-1]])


def main():
    #Intialize MTI Xsens Driver Interface
    
    fileNum = 1
    for incline in treadmillIncline:
        for speed in treadmillSpeed:
            driver = XSensDriver(Filter=True)
            fileName = "{}/{}_{}_{}_{}.csv".format(folder,fileNum,subject[0],speed,incline)
            driver.file_setup(fileName)
            print("Ready------------------------")
            driver.spin()   
            print("The data was sampled {} times".format(driver.count))

            for i in range(driver.count):
                print("Delta_T: {:.5f}\tRoll Angle: {}".format(driver.delta_t[i]/1000,driver.roll[i]))
            fileNum += 1

if __name__ == '__main__':
    main()
