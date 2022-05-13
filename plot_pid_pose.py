import os
from os import listdir
import matplotlib.pyplot as plt
import numpy as np




plots = {}

for file in listdir():
    # read .txt files
    if file.endswith('.txt'):
        name = file[:-4]
        plots[name] = []

        txt = open(file,'r')        
        lines = txt.readlines()
        for l in lines:
            if float(l) != 0:
                plots[name].append(float(l))

# step 1,3,5 - linear
# ref = [0,0,1.5,0,-2,1.5] 
# for i in [1,3,5]:
#     plot_y = 'y' + str(i)
#     plot_yu = 'y' + str(i) + 'u'
#     plot_l = 'l' + str(i)
#     plot_lu = 'l' + str(i) + 'u'

#     plt.figure(i)
#     plt.subplot(2,2,1)
#     plt.plot(list(range(len(plots[plot_y]))),plots[plot_y])
#     plt.axhline(ref[i-1],color='grey')
#     plt.title(plot_y)
#     plt.grid()

#     plt.subplot(2,2,3)
#     plt.plot(list(range(len(plots[plot_yu]))),plots[plot_yu])
#     plt.title(plot_yu)
#     plt.grid()

#     plt.subplot(2,2,2)
#     plt.plot(list(range(len(plots[plot_l]))),plots[plot_l])
#     plt.axhline(ref[i],color='grey')
#     plt.title(plot_l)
#     plt.grid()

#     plt.subplot(2,2,4)
#     plt.plot(list(range(len(plots[plot_lu]))),plots[plot_lu])
#     plt.title(plot_lu)
#     plt.grid()



# # # step 3 - linear
# # plt.figure(2)
# # plt.subplot(2,2,1)
# # plt.plot(list(range(len(plots['y3']))),plots['y3'])
# # plt.axhline(1.5,color='grey')
# # plt.title('y3')
# # plt.grid()

# # plt.subplot(2,2,3)
# # plt.plot(list(range(len(plots['y3u']))),plots['y3u'])
# # plt.title('y3u')
# # plt.grid()

# # plt.subplot(2,2,2)
# # plt.plot(list(range(len(plots['l3']))),plots['l3'])
# # plt.axhline(0,color='grey')
# # plt.title('l3')
# # plt.grid()

# # plt.subplot(2,2,4)
# # plt.plot(list(range(len(plots['l3u']))),plots['l3u'])
# # plt.title('l3u')
# # plt.grid()



# # step 2 and 4 - angular 
# plt.figure(2)
# plt.subplot(2,2,1)
# plt.plot(list(range(len(plots['y2']))),plots['y2'])
# plt.axhline(90,color='grey')
# plt.title('y2')
# plt.grid()

# plt.subplot(2,2,3)
# plt.plot(list(range(len(plots['y2u']))),plots['y2u'])
# plt.title('y2u')
# plt.grid()

# plt.subplot(2,2,2)
# plt.plot(list(range(len(plots['y4']))),plots['y4'])
# plt.axhline(180,color='grey')
# plt.title('y4')
# plt.grid()

# plt.subplot(2,2,4)
# plt.plot(list(range(len(plots['y4u']))),plots['y4u'])
# plt.title('y4u')
# plt.grid()
        


# plt.show()


# ---------------------------------------------------------------

px = []; py = []; pz = []

for i in range(5):
    xname = 'x' + str(i)
    yname = 'y' + str(i)
    zname = 'z' + str(i)

    px += plots[xname]
    py += plots[yname]
    pz += plots[zname]

x0 = plots['x0'][0]
y0 = plots['y0'][0]


plt.figure(4)
plt.plot([x0,x0],[y0,2.6],'r--')
plt.plot([x0,0],[2.6,2.6],'r--')
plt.plot([0,0],[2.6,0],'r--')

plt.plot(px,py)
plt.title('position')
plt.xlabel('x position [m]')
plt.ylabel('y position [m]')
plt.grid()


# error_pos = plots['l1'] + plots['l3'] + [i-1.5 for i in plots['l5']]  

# plt.subplot(1,2,2)
# plt.plot(list(range(len(error_pos))),error_pos)
# plt.title('error')
# plt.grid()
# plt.axvline(len(plots['y1']),color='grey')
# # plt.axvline(len(plots['y1'])+len(plots['y2']),color='grey')
# plt.axvline(len(plots['y1'])+len(plots['y2'])+len(plots['y3']),color='grey')
# # plt.axvline(len(plots['y1'])+len(plots['y2'])+len(plots['y3'])+len(plots['y4']),color='grey')


# pa = []

# pa_ = open('pa.txt','r')
# lines = pa_.readlines()
# for l in lines:
#     if float(l) > -10:
#         pa.append(float(l))
#     else:
#         pa.append(360+float(l))



# plt.figure(5)
# plt.subplot(1,2,1)
# plt.plot(list(range(len(pa))),pa)
# plt.hlines(90,0,len(pa),linestyles='dashed',colors='r')
# plt.hlines(180,0,len(pa),linestyles='dashed',colors='r')
# plt.grid()
# # plt.legend('actual', 'ref')
# plt.title('angle')

# plt.axvline(len(plots['y1']),color='grey')
# plt.axvline(len(plots['y1'])+len(plots['y2']),color='grey')
# plt.axvline(len(plots['y1'])+len(plots['y2'])+len(plots['y3']),color='grey')
# plt.axvline(len(plots['y1'])+len(plots['y2'])+len(plots['y3'])+len(plots['y4']),color='grey')

# error_ang1 = pa[0:len(plots['y1'])]
# error_ang2_ = pa[len(plots['y1'])+len(plots['y2']):len(plots['y1'])+len(plots['y2'])+len(plots['y3'])]
# error_ang2 =[i-90 for i in error_ang2_]
# error_ang3_ = pa[len(plots['y1'])+len(plots['y2'])+len(plots['y3'])+len(plots['y4']) : len(plots['y1'])+len(plots['y2'])+len(plots['y3'])+len(plots['y4'])+len(plots['y5'])]
# error_ang3 =[i-180 for i in error_ang3_]

# error_ang = error_ang1 + [0]*len(plots['y2']) + error_ang2 + [0]*len(plots['y4']) + error_ang3
# plt.subplot(1,2,2)
# plt.plot(list(range(len(error_ang))),error_ang)
# plt.grid()
# plt.title('error')
# plt.axvline(len(plots['y1']),color='grey')
# plt.axvline(len(plots['y1'])+len(plots['y2']),color='grey')
# plt.axvline(len(plots['y1'])+len(plots['y2'])+len(plots['y3']),color='grey')
# plt.axvline(len(plots['y1'])+len(plots['y2'])+len(plots['y3'])+len(plots['y4']),color='grey')


plt.show()
