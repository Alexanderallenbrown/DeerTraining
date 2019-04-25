from numpy import *
from matplotlib import *
### CREATE ANIMATION
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

#Gfname = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa) + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/trialData/generation' + str(generation) + 'ID_' + str(ID) + '/trial_' + str(trial) + '.txt';
Gfname = 'GenerationFiles/TestTest.txt'


print Gfname

#should have an array of size m*h (of object values )

#in some way, read in a text file to fill an array
# Create car vectors to be used
car_x = []
car_y = []
car_yaw = []

# Create deer vectors to be used
deer_x = []
deer_y = []
deer_yaw = []

with open(Gfname, "r") as ins:
    for line in ins:
        values = line.split()
        #print values
        #print values
        car_x.append(float(values[5]))
        car_y.append(float(values[3]))
        car_yaw.append(float(values[7]))
        deer_x.append(float(values[17]))
        deer_y.append(float(values[18]))
        deer_yaw.append(float(values[16]))

print deer_x
print deer_y

factor = 1

car_xs = []
car_ys = []
car_yaws = []

# Create deer vectors to be used
deer_xs = []
deer_ys = []
deer_yaws = []

for ind in range(0,5):
    ind1 = ind * 120
    car_xs.append(car_x[ind1])
    car_ys.append(car_y[ind1])
    car_yaws.append(car_yaw[ind1])
    deer_xs.append(deer_x[ind1])
    deer_ys.append(deer_y[ind1])


deer_length = 1.5 # meters
deer_width = 0.5 # meters
car_length = 4.5 # meters
car_width = 2.0 # meters

fig = plt.figure(figsize=(6.4*factor,4.8*factor))
ax = fig.add_subplot(111)
plt.axis('equal')
ax.set_xlim(20, 140)
ax.set_ylim(-25, 25)

background = patches.Rectangle((-100,-100),10000,10000,fc='g')
car_circle = patches.Circle((100,0),60,fc = 'w', alpha = 0.25)
car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='b', alpha = 0.5)
deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='g', alpha = 0.5)
road = patches.Rectangle((-100,-3.75),10000,11,fc='k')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='y')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='y')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='w')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='w')


car1 = patches.Rectangle(([car_xs[0]-(car_length/2), car_ys[0]-(car_width/2)]), car_length, car_width,angle = car_yaws[0], fc='b', alpha = 0.5)
car2 = patches.Rectangle(([car_xs[1]-(car_length/2), car_ys[1]-(car_width/2)]), car_length, car_width,angle = car_yaws[1], fc='b', alpha = 0.5)
car3 = patches.Rectangle(([car_xs[2]-(car_length/2), car_ys[2]-(car_width/2)]), car_length, car_width,angle = car_yaws[2], fc='b', alpha = 0.5)
car4 = patches.Rectangle(([car_xs[3]-(car_length/2), car_ys[3]-(car_width/2)]), car_length, car_width,angle = car_yaws[3], fc='b', alpha = 0.5)
car5 = patches.Rectangle(([car_xs[4]-(car_length/2), car_ys[4]-(car_width/2)]), car_length, car_width,angle = car_yaws[4], fc='b', alpha = 0.5)




ax.add_patch(background)
ax.add_patch(road)
ax.add_patch(left_line)
ax.add_patch(right_line)
ax.add_patch(center_line_1)
ax.add_patch(center_line_2)
ax.add_patch(car1)
ax.add_patch(car2)
ax.add_patch(car3)
ax.add_patch(car4)
ax.add_patch(car5)



plt.show()



