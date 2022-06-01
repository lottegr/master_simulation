import os
from os import listdir
import matplotlib.pyplot as plt
import numpy as np




plots = {}

dirs = ['14']

for dir in dirs:
    for file in listdir(dir):
        # read .txt files
        if file.endswith('.txt'):
            name = file[:-4]
            plots[name] = []

            txt = open(dir + "/" + file,'r')        
            lines = txt.readlines()
            for l in lines:
                if abs(float(l)) > 1e-100:
                    if name.startswith('z') and float(l) < -150:
                        plots[name].append(float(l)+360)
                    else:
                        plots[name].append(float(l))






    # ------------------------- plot pose -----------------------------------

    px = []; py = []; pz = []; t = [0]; tp = 0

    for i in range(5):
        xname = 'x' + str(i)
        yname = 'y' + str(i)
        zname = 'z' + str(i)

        px += plots[xname]
        py += plots[yname]
        pz += plots[zname]

        tp += len(plots[xname])
        t.append(tp)

        # print(i, "-", len(plots[xname]), "-", len(plots[yname]))

    x0 = plots['x0'][0]
    y0 = plots['y0'][0]


    plt.figure(1)
    # u-turn
    plt.plot([-2.6,-2.6],[y0,2.6],'r--', label='reference')
    plt.plot([-2.6,0],[2.6,2.6],'r--')
    plt.plot([0,0],[2.6,0],'r--')

    # # exit-enter
    # plt.plot([0,0],[y0,3.5],'r--', label='reference')
    # plt.plot([0,2],[3.5,3.5],'r--')
    # plt.plot([2,2],[3.5,4.5],'r--')
    # plt.plot([2,1.25],[4.5,4.5],'r--')
    # plt.plot([1.25,1.25],[4.5,2.5],'r--')
    # plt.plot([1.25,0],[2.5,2.5],'r--')
    # plt.plot([0,0],[2.5,0],'r--')

    # # u-turn sim
    # plt.plot([0,1.5],[0,0],'r--', label='reference')
    # plt.plot([1.5,1.5],[0,2],'r--')
    # plt.plot([1.5,0],[2,2],'r--')

    plt.plot(px,py, label='measurement')
    plt.title('Position')
    plt.xlabel('x position [m]')
    plt.ylabel('y position [m]')
    plt.legend()
    plt.grid()


    # ---------------------------- orientation

    plt.figure(2)
    # u-turn
    plt.plot([0,t[1]],[90,90],'r--', label='reference')
    plt.plot([t[2],t[3]],[0,0],'r--')
    plt.plot([t[4],t[5]],[-90,-90],'r--')

    # # exit-enter
    # plt.plot([0,t[1]],[90,90],'r--', label='reference')
    # plt.plot([t[2],t[3]],[0,0],'r--')
    # plt.plot([t[4],t[5]],[90,90],'r--')
    # plt.plot([t[6],t[7]],[0,0],'r--')
    # plt.plot([t[8],t[9]],[90,90],'r--')
    # plt.plot([t[10],t[11]],[0,0],'r--')
    # plt.plot([t[12],t[13]],[-90,-90],'r--')

    # # u-turn sim
    # plt.plot([0,t[1]],[0,0],'r--', label='reference')
    # plt.plot([t[2],t[3]],[90,90],'r--')
    # plt.plot([t[4],t[5]],[180,180],'r--')


    for i in range(len(t)):
        plt.axvline(t[i],color='grey')
    plt.plot(range(len(pz)),pz, label='measurement')
    plt.title('Orientation')
    plt.xlabel('x')
    plt.ylabel('orientation [deg]')
    plt.legend()
    plt.grid()





    # ---------------------------- plot error ---------------------------------

    plt.figure(3)
    # u-turn
    plt.plot(range(t[0],t[1]), [i-(-2.6) for i in plots['x0']], 'purple', label='x')
    plt.plot(range(t[1],t[2]), [i-(-2.6) for i in plots['x1']], 'purple')
    plt.plot(range(t[1],t[2]), [i-2.6 for i in plots['y1']], 'g', label='y')
    plt.plot(range(t[2],t[3]), [i-2.6 for i in plots['y2']], 'g')
    plt.plot(range(t[3],t[4]), [i-2.6 for i in plots['y3']], 'g')
    plt.plot(range(t[3],t[4]), [i-0 for i in plots['x3']], 'purple')
    plt.plot(range(t[4],t[5]), [i-0 for i in plots['x4']], 'purple')

    # # exit-enter
    # plt.plot(range(t[0],t[1]), [i-0 for i in plots['x0']], 'purple', label='x')
    # plt.plot(range(t[1],t[2]), [i-0 for i in plots['x1']], 'purple')
    # plt.plot(range(t[1],t[2]), [i-3.5 for i in plots['y1']], 'g', label='y')
    # plt.plot(range(t[2],t[3]), [i-3.5 for i in plots['y2']], 'g')
    # plt.plot(range(t[3],t[4]), [i-3.5 for i in plots['y3']], 'g')
    # plt.plot(range(t[3],t[4]), [i-2 for i in plots['x3']], 'purple')
    # plt.plot(range(t[4],t[5]), [i-2 for i in plots['x4']], 'purple')
    # plt.plot(range(t[5],t[6]), [i-2 for i in plots['x5']], 'purple')
    # plt.plot(range(t[5],t[6]), [i-4.5 for i in plots['y5']], 'g')
    # plt.plot(range(t[6],t[7]), [i-4.5 for i in plots['y6']], 'g')
    # plt.plot(range(t[7],t[8]), [i-4.5 for i in plots['y7']], 'g')
    # plt.plot(range(t[7],t[8]), [i-1.25 for i in plots['x7']], 'purple')
    # plt.plot(range(t[8],t[9]), [i-1.25 for i in plots['x8']], 'purple')
    # plt.plot(range(t[9],t[10]), [i-1.25 for i in plots['x9']], 'purple')
    # plt.plot(range(t[9],t[10]), [i-2.5 for i in plots['y9']], 'g')
    # plt.plot(range(t[10],t[11]), [i-2.5 for i in plots['y10']], 'g')
    # plt.plot(range(t[11],t[12]), [i-2.5 for i in plots['y11']], 'g')
    # plt.plot(range(t[11],t[12]), [i-0 for i in plots['x11']], 'purple')
    # plt.plot(range(t[12],t[13]), [i-0 for i in plots['x12']], 'purple')



    # # u-turn sim
    # plt.plot(range(t[0],t[1]), [i-0 for i in plots['y0']], 'g')
    # plt.plot(range(t[1],t[2]), [i-0 for i in plots['y1']], 'g')
    # plt.plot(range(t[1],t[2]), [i-1.5 for i in plots['x1']], 'purple', label='x')
    # plt.plot(range(t[2],t[3]), [i-1.5 for i in plots['x2']], 'purple')
    # plt.plot(range(t[3],t[4]), [i-1.5 for i in plots['x3']], 'purple')
    # plt.plot(range(t[3],t[4]), [i-2 for i in plots['y3']], 'g', label='y')
    # plt.plot(range(t[4],t[5]), [i-2 for i in plots['y4']], 'g')







    for i in range(len(t)):
        plt.axvline(t[i],color='grey')
    plt.axhline(0,color='grey', linestyle="--")
    plt.title('Deviation from goal position')
    plt.ylabel('Deviation [m]')
    plt.legend()
    plt.grid()





plt.show()
