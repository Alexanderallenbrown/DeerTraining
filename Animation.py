from numpy import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

TestNumber = 1
FileName ='Test/Test' + str(TestNumber) + '.txt';

lines = [line.rstrip('\n') for line in open(FileName)]

car_y = lines[0]
car_v = lines[1]
car_x = lines[2]
car_u = lines[3]
car_psi = lines[4]
car_r = lines[5]

deer_speed = lines[6]
deer_psi = lines[7]
deer_x = lines[8]
deer_y = lines[9]

# Define parameters
deer_length = 1.5 # meters
deer_width = 0.5 # meters
car_length = 4.5 # meters
car_width = 2.0 # meters

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111)
plt.axis('equal')
ax.set_xlim(0, 100)
ax.set_ylim(-25, 25)

# Initialize rectangles
car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='k')
deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='y')

def init():
    ax.add_patch(car_plot)
    ax.add_patch(deer_plot)
    return car_plot,deer_plot,

print cos(car_psi[0])

# Set animation
def animate(i):
    car_plot.set_width(car_length)
    car_plot.set_height(car_width)
    car_plot.set_xy([car_x[i]-(car_length/2*cos(car_psi[i])), car_y[i]-(car_width/2*sin(car_psi[i]))])
    car_plot.angle = car_psi[i]*180/3.14

    deer_plot.set_width(deer_length)
    deer_plot.set_height(deer_width)
    deer_plot.set_xy([deer_x[i]-(deer_length/2*sin(deer_psi[i])), deer_y[i]]-(deer_width/2*cos(deer_psi[i])))
    deer_plot.angle = 90-deer_psi[i]*180/3.14

    return car_plot,deer_plot,

# Run anumation
anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x),interval=50,blit=True)

### ANIMATION END

plt.show()
