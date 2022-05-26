import matplotlib.pyplot as plt

x = [-2.6, -2.6, 0, 0]
y = [1, 2.6, 2.6, 0]

plt.figure(1)
for i in range(len(x)-1):
    if i%2 != 0:
        plt.arrow(x[i], y[i], x[i+1]-x[i]-0.1, y[i+1]-y[i], width=0.02)
    else:
        plt.arrow(x[i], y[i], x[i+1]-x[i], y[i+1]-y[i]-0.1, width=0.02)

plt.xlabel('x position [m]')
plt.ylabel('y position [m]')
plt.grid()

# -----------------------------------------------------------------------------------

x = [-0.01, -0.01, 2.5, 2.5, 1.25, 1.25, 0.01, 0.01]
y = [-1, 2.5, 2.5, 3.5, 3.5, 1.5, 1.5, -0.5]

plt.figure(2)
for i in range(len(x)-1):
    if i%2 != 0:
        if i < 3:
            plt.arrow(x[i], y[i], x[i+1]-x[i]-0.1, y[i+1]-y[i], width=0.02)
        else: 
            plt.arrow(x[i], y[i], x[i+1]-x[i]+0.1, y[i+1]-y[i], width=0.02)
    elif i < 4:
        plt.arrow(x[i], y[i], x[i+1]-x[i], y[i+1]-y[i]-0.1, width=0.02)
    else:
        plt.arrow(x[i], y[i], x[i+1]-x[i], y[i+1]-y[i]+0.1, width=0.02)
    
plt.xlabel('x position [m]')
plt.ylabel('y position [m]')
plt.grid()
plt.show()