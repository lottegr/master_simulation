import os
from os import listdir
import matplotlib.pyplot as plt
import numpy as np

# ------------


lab = ["P =1", "P = 0.5", "P = 0.25"]
j = -1
# for i in range(5,10):
for i in [4,9]:
    j += 1
    plot_nr = str(i)
    ref = 0

    xlab = 'Position [m]'
    ulab = 'Angular velocity [rad/s]'

    # xlab = 'Position [m]'
    # ulab = 'Velocity [m/s]'

    # ------------

    x = []; u = []; p = []
    x_file = "sensor" + plot_nr + ".txt"
    u_file = "input" + plot_nr + ".txt"

    # x_file = "pose" + plot_nr + ".txt"
    # u_file = "input" + plot_nr + ".txt"
    # p_file = "sensor" + plot_nr + ".txt"

    x_ = open(x_file,'r')
    u_ = open(u_file,'r')
    # p_ = open(p_file,'r')
    lines_x = x_.readlines()
    lines_u = u_.readlines()
    # lines_p = p_.readlines()
    for l in lines_x:
        if abs(float(l)) > 1e-100:
            x.append(float(l)-1)
    for l in lines_u:
        if abs(float(l)) > 1e-100:
            u.append(float(l))
    # for l in lines_p:
    #     if abs(float(l)) > 1e-100:
    #         p.append(float(l)-1)


    # plt.figure(i)
    plt.subplot(1,2,1)
    # plt.plot(range(len(x)),x, label=lab[j])
    plt.plot(range(len(x)),x, label="Pose of base")
    # plt.plot(range(len(p)),p, label="Pose of p")
    plt.ylabel(xlab)
    plt.axhline(ref,color='grey')
    plt.title('Measurements')
    # plt.grid()

    plt.subplot(1,2,2)
    plt.plot(range(len(u)),u)
    plt.ylabel(ulab)
    plt.title('Control variable')


plt.grid()

plt.subplot(1,2,1)
plt.legend()
plt.grid()
plt.show()


