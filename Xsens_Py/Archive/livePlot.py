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

        self.graphWidget = pg.PlotWidget()
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        #self.x = list(range(100))  # 100 time points
        #self.y = [randint(0,100) for _ in range(100)]  # 100 data points

        self.x = []
        self.roll = []
        self.pitch = []

        self.graphWidget.setBackground('w')
        self.graphWidget.setTitle("Roll and Pitch Angle in Degrees", color="k", size="12pt")
        self.graphWidget.setLabel('left', 'Angle [Deg] (°)')
        self.graphWidget.setLabel('bottom', 'Time [sec] (s)')
        self.graphWidget.addLegend()

        pen1 = pg.mkPen(color=(255, 0, 0))
        self.data_line1 =  self.graphWidget.plot(self.x, self.roll, pen=pen1, name="Roll Angle")
        pen2 = pg.mkPen(color=(0, 255, 0))
        self.data_line2 =  self.graphWidget.plot(self.x, self.pitch, pen=pen2, name="Pitch Angle")

        

        self.timer = QtCore.QTimer()
        self.timer.setInterval(0)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        self.driver.spin_once()
        self.driver.count = self.driver.count + 1

        if len(self.x) == 200:
            self.x = self.x[1:]
            self.roll = self.roll[1:]
            self.pitch = self.pitch[1:]

        self.x.append(self.driver.delta_t_curr/1000)
        self.roll.append(self.driver.roll_cur)
        self.pitch.append(self.driver.pitch_cur)

        self.data_line1.setData(self.x, self.roll)  # Update the data.
        self.data_line2.setData(self.x, self.pitch)  # Update the data.

def main():
    '''Create a ROS node and instantiate the class.'''
    #rospy.init_node('xsens_driver')
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
    


if __name__ == '__main__':
    main()


