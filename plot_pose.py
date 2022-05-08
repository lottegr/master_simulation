import os
from os import listdir
import matplotlib.pyplot as plt
import numpy as np




plots = {}
plotnames_x = []
plotnames_y = []
plotnames_rot = []

for file in listdir():
    # read .txt files
    if file.endswith('_x.txt'):
        name = file[:-4]
        plots[name] = []
        plotnames_x.append(name)

        txt = open(file,'r')        
        lines = txt.readlines()
        for l in lines:
            plots[name].append(float(l))

    elif file.endswith('_y.txt'):
        name = file[:-4]
        plots[name] = []
        plotnames_y.append(name)

        txt = open(file,'r')        
        lines = txt.readlines()
        for l in lines:
            plots[name].append(float(l))

    elif file.endswith('_rot.txt'):
        name = file[:-4]
        plots[name] = []
        plotnames_rot.append(name)

        txt = open(file,'r')        
        lines = txt.readlines()
        for l in lines:
            if float(l) > -100:
                plots[name].append(float(l))
            else:
                plots[name].append(360+float(l))

time = range(len(plots['odom_x']))

plots['imu_rot'][0:2] = [0,0]

for name in plotnames_x:  
    plt.figure(1)
    plt.plot(time,plots[name],label=name)
plt.title('pose_x')
plt.legend()
plt.grid()

for name in plotnames_y:  
    plt.figure(2)
    plt.plot(time,plots[name],label=name)
plt.title('pose_y')
plt.legend()
plt.grid()

for name in plotnames_rot:  
    plt.figure(3)
    plt.plot(time,plots[name],label=name)
plt.title('pose_rot')
plt.legend()
plt.grid()

plt.show()

