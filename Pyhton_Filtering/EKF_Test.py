#Tim Driscoll
#2/1/23

#Sample sinusoidal data set for testing filtering techniques

import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin

_TESTDATA = 'sin-data.txt'

class FilterTest(object):

    def __init__(self, sampleData=_TESTDATA):

        self.noise_data = []
        self.true_data = []
        self.count = []

        i = 0

        with open(sampleData, 'r') as file:
            lines = file.readlines()
        
        for row in lines:
            row = row.strip('\n')
            true_point, noise_point = row.split(' ')
            print("True: {}\tNoise: {}".format(true_point, noise_point))
            self.noise_data.append(float(noise_point))
            self.true_data.append(float(true_point))
            self.count.append(i)
            i += 1
    
    def plot_data(self,X,Y,title,last=False):

        plt.figure()
        plt.plot(X, Y, 'k-')

        plt.axis([0, 800, -5, 5])
        plt.title(title)
        plt.xlabel('Time [secs]')
        plt.ylabel('Angle [degs]')


        if last:
            plt.show()
        else:
            plt.show(block=False)
    
    def EKF(self, measurements):

        estimates= []

        #Define the state variables and covariance 
        X = np.mat([[0], [0], [measurements[0]]])
        S = np.mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        I = np.mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        Xt_t1 = np.mat([[0.0], [0.0], [0.0]])
        Kt = np.mat([[0], [0], [0]])
        St_t1 = np.mat([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

        #Define the Jacobians
        J_dfdx = np.mat([[1, 1, 0], [0, 1, 0], [(1/10)*cos(X[0,0]/10), 0, 0]])
        J_dfda = np.mat([[0, 0, 0], [0, 1, 0], [0, 0, 0]])
        J_dgdx = np.mat([0, 0, 1])
        J_dgdn = np.mat([1])

        #Define the noise covariance 
        # Dynamic noise covariance (how much prediction is trusted (0 is no noise))
        Q = np.mat([[0, 0, 0], [0, 0.001, 0], [0, 0, 0]])
        # Measurment Noise
        R = np.mat([1])

        # Loop to perform EKF Estimation
        for tt in range(len(measurements)):
            J_dfdx[2,0] = (1/10)*cos(X[0,0]/10) # Update The Jacobians

            Xt_t1[0,0] = X[0,0] + X[1,0]
            Xt_t1[1,0] = X[1,0]
            Xt_t1[2,0] = sin(X[0,0]/10)

            St_t1 = J_dfdx*S*J_dfdx.transpose() + J_dfda*Q*J_dfda.transpose()
            Yt = measurements[tt]
            Kt = (St_t1*J_dgdx.getT()) * pow(J_dgdx*St_t1*J_dgdx.getT() + J_dgdn*R*J_dgdn.getT(), -1)

            g = sin(X[0,0]/10)
            X = Xt_t1 + Kt*(Yt - g)
            S = (I - Kt*J_dgdx)*St_t1

            estimates.append(X[2,0])
        
        return estimates


def main():
    
    test = FilterTest()

    test.plot_data(test.count, test.noise_data, "Noisy Input Signal")
    test.plot_data(test.count, test.true_data, "Ground Truth Signal")

    estimates = test.EKF(test.noise_data)

    test.plot_data(test.count, estimates, "Filtered Input Signal", True)


if __name__ == '__main__':
    main()


