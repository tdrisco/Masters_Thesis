#!/usr/bin/env python

import MTI_Setup
import MID_Codes

import time
import datetime

from math import sqrt

_RUNTIME = 1



class XSensDriver(object):

    def __init__(self):

        #Tune these parameters depeding on Pi or computer
        device = 'COM10'
        baudrate = 115200
        timeout = 0.002

        self.count = 0


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
        output_config = MTI_Setup.get_output_config('if2000,oe400fw')
        print("Changing output configuration")
        self.mt.SetOutputConfiguration(output_config)
        print("System is Ok, Ready to Record.")

    def spin(self):
        try:
            t_end = time.time() + _RUNTIME
            while time.time() < t_end:
                # Spin to try to get new messages
                self.count = self.count + 1
                self.spin_once()
                #self.reset_vars()
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
                print('Euler Angles - Roll: '+str(o['Roll'])+', Pitch: '+str(o['Pitch'])+',y='+str(o['Yaw']))
            except KeyError:
                pass

        def fill_from_Acceleration(o):
            '''Fill messages with information from 'Acceleration' MTData2 block.'''
            fill_from_Timestamp(o)
            print('acceleration_data x='+str(o['accX'])+',y='+str(o['accY'])+',z='+str(o['accZ']))
            pass

        def fill_from_Angular_Velocity(o):
            '''Fill messages with information from 'Angular Velocity' MTData2 block.'''
            print('angular_vel_data x='+str(o['gyrX'])+',y='+str(o['gyrY'])+',z='+str(o['gyrZ']))
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
            time.sleep(0.001)
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


def main():
    '''Create a ROS node and instantiate the class.'''
    #rospy.init_node('xsens_driver')
    driver = XSensDriver()
    driver.spin()
    print("The data was sampled {} times".format(driver.count))
    


if __name__ == '__main__':
    main()
