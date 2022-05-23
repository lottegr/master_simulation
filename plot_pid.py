import os
from os import listdir
import matplotlib.pyplot as plt
import numpy as np

# ------------

for i in range(1,5):
    plot_nr = str(i)
    ref = 0

    xlab = ''
    ulab = 'Velocity [m/s]'

    # xlab = 'Position [m]'
    # ulab = 'Velocity [m/s]'

    # ------------

    x = []; u = []
    x_file = "sensor" + plot_nr + ".txt"
    u_file = "input" + plot_nr + ".txt"

    x_ = open(x_file,'r')
    u_ = open(u_file,'r')
    lines_x = x_.readlines()
    lines_u = u_.readlines()
    for l in lines_x:
        if abs(float(l)) > 1e-100:
            x.append(float(l))
    for l in lines_u:
        if abs(float(l)) > 1e-100:
            u.append(float(l))


    plt.figure(i)
    plt.subplot(1,2,1)
    plt.plot(range(len(x)),x)
    plt.ylabel(xlab)
    plt.axhline(ref,color='grey')
    plt.title('Measurement')
    plt.grid()

    plt.subplot(1,2,2)
    plt.plot(range(len(u)),u)
    plt.ylabel(ulab)
    plt.title('Control variable')
    plt.grid()
plt.show()


