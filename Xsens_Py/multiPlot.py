from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint

import MTI_Output as MTI

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        self.driver = MTI.XSensDriver()

        super(MainWindow, self).__init__(*args, **kwargs)

        self.win = pg.GraphicsLayoutWidget(show=True, title="Real-Time Data GUI")
        self.win.resize(600,500)
        #self.win.setWindowTitle('Real-Time Data Visualization')

        self.x = []
        self.roll = []
        self.pitch = []
        self.angVel = []
        self.phaseVar = []

        self.win.setBackground('w')
        pg.setConfigOptions(antialias=True)

        self.p1 = self.win.addPlot(title="Roll and Pitch Angle")
        #self.p2 = self.win.addPlot(title="Phase Portrait")
        #self.p3 = self.win.addPlot(title="Phase Variable")

        self.p1.setLabel('left', 'Angle [Deg] (°)')
        self.p1.setLabel('bottom', 'Time [sec] (s)')

        #self.p2.setLabel('left', 'Thigh Velocity [Deg/sec] (°/s)')
       # self.p2.setLabel('bottom', 'Thigh Angle [Deg] (°)')

        #self.p3.setLabel('left', 'Phase Variable')
       # self.p3.setLabel('bottom', 'Time [sec] (s)')

        self.p1.addLegend()

        pen1 = pg.mkPen(color=(255, 0, 0))
        self.data_line1 =  self.p1.plot(self.x, self.roll, pen=pen1, name="Roll Angle")
        pen2 = pg.mkPen(color=(0, 0, 255))
        self.data_line2 =  self.p1.plot(self.x, self.pitch, pen=pen2, name="Pitch Angle")

        pen3 = pg.mkPen(color=(0, 255, 0))
        #self.data_line3 =  self.p2.plot(self.roll, self.angVel, pen=pen3, name="Phase Portrait")

        pen4 = pg.mkPen(color=(0, 0, 0))
        #self.data_line4 =  self.p3.plot(self.x, self.phaseVar, pen=pen4, name="Phase Variable")
        
        self.timer = QtCore.QTimer()
        self.timer.setInterval(0)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        self.driver.spin_once()
        self.driver.count = self.driver.count + 1

        if len(self.x) == 100:
            self.x = self.x[1:]
            self.roll = self.roll[1:]
            self.pitch = self.pitch[1:]
            self.angVel = self.angVel[1:]
            self.phaseVar = self.phaseVar[1:]

        self.x.append(self.driver.delta_t_curr/1000)
        self.roll.append(self.driver.roll_cur)
        self.pitch.append(self.driver.pitch_cur)
        self.angVel.append(self.driver.angVel_cur)
        self.phaseVar.append(self.driver.phaseVar_cur)

        self.data_line1.setData(self.x, self.roll)  # Update the data.
        self.data_line2.setData(self.x, self.pitch)  # Update the data.
       # self.data_line3.setData(self.roll, self.angVel)  # Update the data.
       # self.data_line4.setData(self.x, self.phaseVar)  # Update the data.

def main():
    '''Create a ROS node and instantiate the class.'''
    #rospy.init_node('xsens_driver')
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    #w.show()
    sys.exit(app.exec_())
    


if __name__ == '__main__':
    main()


