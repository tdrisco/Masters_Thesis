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
        self.setCentralWidget(self.graphWidget)

        #self.x = list(range(100))  # 100 time points
        #self.y = [randint(0,100) for _ in range(100)]  # 100 data points

        self.x = []
        self.y = []

        self.graphWidget.setBackground('w')
        self.graphWidget.setTitle("Roll Angle in Degrees", color="k", size="12pt")
        self.graphWidget.setLabel('left', 'Angle [Deg] (°)')
        self.graphWidget.setLabel('bottom', 'Time [sec] (s)')

        pen = pg.mkPen(color=(255, 0, 0))
        self.data_line =  self.graphWidget.plot(self.x, self.y, pen=pen)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        self.driver.spin_once()
        self.driver.count = self.driver.count + 1

        #self.x = self.x[1:]  # Remove the first y element.
        #self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.
        self.x.append(self.driver.delta_t_curr/1000)

        #self.y = self.y[1:]  # Remove the first
        self.y.append(self.driver.roll_cur)

        #self.y = self.y[1:]  # Remove the first
        #self.y.append(randint(0,100))  # Add a new random value.

        self.data_line.setData(self.x, self.y)  # Update the data.

def main():
    '''Create a ROS node and instantiate the class.'''
    #rospy.init_node('xsens_driver')
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
    


if __name__ == '__main__':
    main()


