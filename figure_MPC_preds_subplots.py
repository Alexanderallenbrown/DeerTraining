from numpy import *
from matplotlib import *
### CREATE ANIMATION
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

#Gfname = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa) + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/trialData/generation' + str(generation) + 'ID_' + str(ID) + '/trial_' + str(trial) + '.txt';
Gfname = 'Test/Test1.txt'
Predname = 'Test/pred.txt'




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
        #print line
        car_x.append(float(values[5]))
        car_y.append(float(values[3]))
        car_yaw.append(float(values[7]))
        deer_x.append(float(values[17]))
        deer_y.append(float(values[18]))
        deer_yaw.append(float(values[16]))


xCarPred0 = []
xCarPred1 = []
xCarPred2 = []
xCarPred3 = []
xCarPred4 = []
xCarPred5 = []
xCarPred6 = []
xCarPred7 = []
xCarPred8 = []
xCarPred9 = []

yCarPred0 = []
yCarPred1 = []
yCarPred2 = []
yCarPred3 = []
yCarPred4 = []
yCarPred5 = []
yCarPred6 = []
yCarPred7 = []
yCarPred8 = []
yCarPred9 = []

xDeerPred0 = []
xDeerPred1 = []
xDeerPred2 = []
xDeerPred3 = []
xDeerPred4 = []
xDeerPred5 = []
xDeerPred6 = []
xDeerPred7 = []
xDeerPred8 = []
xDeerPred9 = []

yDeerPred0 = []
yDeerPred1 = []
yDeerPred2 = []
yDeerPred3 = []
yDeerPred4 = []
yDeerPred5 = []
yDeerPred6 = []
yDeerPred7 = []
yDeerPred8 = []
yDeerPred9 = []

with open(Predname, "r") as ins:
    for line in ins:
        values = line.split()
        #print values
        xCarPred0.append(float(values[1]))
        xCarPred1.append(float(values[2]))
        xCarPred2.append(float(values[3]))
        xCarPred3.append(float(values[4]))
        xCarPred4.append(float(values[5]))
        xCarPred5.append(float(values[6]))
        xCarPred6.append(float(values[7]))
        xCarPred7.append(float(values[8]))
        xCarPred8.append(float(values[9]))
        xCarPred9.append(float(values[10])) 

        yCarPred0.append(float(values[11]))
        yCarPred1.append(float(values[12]))
        yCarPred2.append(float(values[13]))
        yCarPred3.append(float(values[14]))
        yCarPred4.append(float(values[15]))
        yCarPred5.append(float(values[16]))
        yCarPred6.append(float(values[17]))
        yCarPred7.append(float(values[18]))
        yCarPred8.append(float(values[19]))
        yCarPred9.append(float(values[20]))

        xDeerPred0.append(float(values[31]))
        xDeerPred1.append(float(values[32]))
        xDeerPred2.append(float(values[33]))
        xDeerPred3.append(float(values[34]))
        xDeerPred4.append(float(values[35]))
        xDeerPred5.append(float(values[36]))
        xDeerPred6.append(float(values[37]))
        xDeerPred7.append(float(values[38]))
        xDeerPred8.append(float(values[39]))
        xDeerPred9.append(float(values[40]))
        
        yDeerPred0.append(float(values[41]))
        yDeerPred1.append(float(values[42]))
        yDeerPred2.append(float(values[43]))
        yDeerPred3.append(float(values[44]))
        yDeerPred4.append(float(values[45]))
        yDeerPred5.append(float(values[46]))
        yDeerPred6.append(float(values[47]))
        yDeerPred7.append(float(values[48]))
        yDeerPred8.append(float(values[49]))
        yDeerPred9.append(float(values[50]))
        
factor = 3

car_xs = []
car_ys = []
car_yaws = []

# Create deer vectors to be used
deer_xs = []
deer_ys = []
deer_yaws = []
xCarPred1s = []
xCarPred2s = []
xCarPred3s = []
xCarPred4s = []
xCarPred5s = []
yCarPred1s = []
yCarPred2s = []
yCarPred3s = []
yCarPred4s = []
yCarPred5s = []
xDeerPred1s = []
xDeerPred2s = []
xDeerPred3s = []
xDeerPred4s = []
xDeerPred5s = []
yDeerPred1s = []
yDeerPred2s = []
yDeerPred3s = []
yDeerPred4s = []
yDeerPred5s = []


for ind in range(0,6):
    ind1 = 55 + ind * 50
    car_xs.append(car_x[ind1])
    car_ys.append(car_y[ind1])
    car_yaws.append(car_yaw[ind1])
    deer_xs.append(deer_x[ind1])
    deer_ys.append(deer_y[ind1])
    deer_yaws.append(deer_yaw[ind1]*180/3.1415)
    xCarPred1s.append(xCarPred0[ind1])
    xCarPred2s.append(xCarPred2[ind1])
    xCarPred3s.append(xCarPred4[ind1])
    xCarPred4s.append(xCarPred6[ind1])
    xCarPred5s.append(xCarPred8[ind1])
    yCarPred1s.append(yCarPred0[ind1])
    yCarPred2s.append(yCarPred2[ind1])
    yCarPred3s.append(yCarPred4[ind1])
    yCarPred4s.append(yCarPred6[ind1])
    yCarPred5s.append(yCarPred8[ind1])
    xDeerPred1s.append(xDeerPred1[ind1])
    xDeerPred2s.append(xDeerPred3[ind1])
    xDeerPred3s.append(xDeerPred5[ind1])
    xDeerPred4s.append(xDeerPred7[ind1])
    xDeerPred5s.append(xDeerPred9[ind1])
    yDeerPred1s.append(yDeerPred1[ind1])
    yDeerPred2s.append(yDeerPred3[ind1])
    yDeerPred3s.append(yDeerPred5[ind1])
    yDeerPred4s.append(yDeerPred7[ind1])
    yDeerPred5s.append(yDeerPred9[ind1])


deer_length = 1.5 # meters
deer_width = 0.5 # meters
car_length = 4.5 # meters
car_width = 2.0 # meters

fig = plt.figure(figsize=(6.4*factor,4.8*factor))
ax = fig.add_subplot(321)
plt.axis('equal')

plt.autoscale(enable=True, axis = 'y', tight=True)
ax.set_title('$t = t_0$')
ax.spines['bottom'].set_color('1')
ax.spines['top'].set_color('1')
ax.spines['left'].set_color('1')
ax.spines['right'].set_color('1')
ax.tick_params(axis ='x', colors ='1')
ax.tick_params(axis ='y', colors ='1')
ax.set_xlim(60, 180)



background = patches.Rectangle((-100,-100),10000,10000,facecolor='1')
road = patches.Rectangle((-100,-3.75),10000,11,fc='0.8')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='0')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='0')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='0')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='0')
trees = patches.Rectangle((-100,-100),10000,90,fc='0.8',alpha=0.5)




car1 = patches.Rectangle(([car_xs[0]-(car_length/2), car_ys[0]-(car_width/2)]), car_length, car_width,angle = car_yaws[0], fc='0')
deer1 = patches.Rectangle(([deer_xs[0]-(deer_length/2), deer_ys[0]-(deer_width/2)]), deer_length, deer_width,angle = 90-deer_yaws[0], fc='0')
 

radius = 1.0

carPred11 = patches.Circle(([xCarPred1s[0],yCarPred1s[0]]),radius, fc ='1')
carPred21 = patches.Circle(([xCarPred2s[0],yCarPred2s[0]]),radius, fc ='1')
carPred31 = patches.Circle(([xCarPred3s[0],yCarPred3s[0]]),radius, fc ='1')
carPred41 = patches.Circle(([xCarPred4s[0],yCarPred4s[0]]),radius, fc ='1')
carPred51 = patches.Circle(([xCarPred5s[0],yCarPred5s[0]]),radius, fc ='1')


deerPred11 = patches.Circle(([xDeerPred1s[0],yDeerPred1s[0]]),radius, fc = '1')
deerPred21 = patches.Circle(([xDeerPred2s[0],yDeerPred2s[0]]),radius, fc = '1')
deerPred31 = patches.Circle(([xDeerPred3s[0],yDeerPred3s[0]]),radius, fc = '1')
deerPred41 = patches.Circle(([xDeerPred4s[0],yDeerPred4s[0]]),radius, fc = '1')
deerPred51 = patches.Circle(([xDeerPred5s[0],yDeerPred5s[0]]),radius, fc = '1')


ax.add_patch(background)
ax.add_patch(road)
ax.add_patch(left_line)
ax.add_patch(right_line)
ax.add_patch(center_line_1)
ax.add_patch(center_line_2)
ax.add_patch(car1)
ax.add_patch(trees)

ax.add_patch(deer1)

ax.add_patch(carPred11)
ax.add_patch(carPred21)
ax.add_patch(carPred31)
ax.add_patch(carPred41)
ax.add_patch(carPred51)

ax.add_patch(deerPred11)
ax.add_patch(deerPred21)
ax.add_patch(deerPred31)
ax.add_patch(deerPred41)
ax.add_patch(deerPred51)



ax = fig.add_subplot(323)
ax.set_title('$t = t_0 + 0.3s$')
plt.axis('equal')
ax.spines['bottom'].set_color('1')
ax.spines['top'].set_color('1')
ax.spines['left'].set_color('1')
ax.spines['right'].set_color('1')
ax.tick_params(axis ='x', colors ='1')
ax.tick_params(axis ='y', colors ='1')
ax.set_xlim(60, 180)

background = patches.Rectangle((-100,-100),10000,10000,facecolor='1')
road = patches.Rectangle((-100,-3.75),10000,11,fc='0.8')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='0')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='0')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='0')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='0')
trees = patches.Rectangle((-100,-100),10000,90,fc='0.8',alpha=0.5)


car2 = patches.Rectangle(([car_xs[1]-(car_length/2), car_ys[1]-(car_width/2)]), car_length, car_width,angle = car_yaws[1], fc='0')
deer2 = patches.Rectangle(([deer_xs[1]-(deer_length/2), deer_ys[1]-(deer_width/2)]), deer_length, deer_width,angle = 90-deer_yaws[1], fc='0')

carPred12 = patches.Circle(([xCarPred1s[1],yCarPred1s[1]]),radius, fc ='1')
carPred22 = patches.Circle(([xCarPred2s[1],yCarPred2s[1]]),radius, fc ='1')
carPred32 = patches.Circle(([xCarPred3s[1],yCarPred3s[1]]),radius, fc ='1')
carPred42 = patches.Circle(([xCarPred4s[1],yCarPred4s[1]]),radius, fc ='1')
carPred52 = patches.Circle(([xCarPred5s[1],yCarPred5s[1]]),radius, fc ='1')
deerPred12 = patches.Circle(([xDeerPred1s[1],yDeerPred1s[1]]),radius, fc = '1')
deerPred22 = patches.Circle(([xDeerPred2s[1],yDeerPred2s[1]]),radius, fc = '1')
deerPred32 = patches.Circle(([xDeerPred3s[1],yDeerPred3s[1]]),radius, fc = '1')
deerPred42 = patches.Circle(([xDeerPred4s[1],yDeerPred4s[1]]),radius, fc = '1')
deerPred52 = patches.Circle(([xDeerPred5s[1],yDeerPred5s[1]]),radius, fc = '1')

ax.add_patch(background)
ax.add_patch(road)
ax.add_patch(left_line)
ax.add_patch(right_line)
ax.add_patch(center_line_1)
ax.add_patch(center_line_2)
ax.add_patch(car2)
ax.add_patch(deer2)
ax.add_patch(trees)


ax.add_patch(carPred12)
ax.add_patch(carPred22)
ax.add_patch(carPred32)
ax.add_patch(carPred42)
ax.add_patch(carPred52)

ax.add_patch(deerPred12)
ax.add_patch(deerPred22)
ax.add_patch(deerPred32)
ax.add_patch(deerPred42)
ax.add_patch(deerPred52)



ax = fig.add_subplot(325)
plt.axis('equal')
ax.set_title('$t = t_0 + 0.6s$')
ax.spines['bottom'].set_color('1')
ax.spines['top'].set_color('1')
ax.spines['left'].set_color('1')
ax.spines['right'].set_color('1')
ax.tick_params(axis ='x', colors ='1')
ax.tick_params(axis ='y', colors ='1')
ax.set_xlim(60, 180)

background = patches.Rectangle((-100,-100),10000,10000,facecolor='1')
road = patches.Rectangle((-100,-3.75),10000,11,fc='0.8')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='0')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='0')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='0')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='0')
trees = patches.Rectangle((-100,-100),10000,90,fc='0.8',alpha=0.5)


car3 = patches.Rectangle(([car_xs[2]-(car_length/2), car_ys[2]-(car_width/2)]), car_length, car_width,angle = car_yaws[2], fc='0')
deer3 = patches.Rectangle(([deer_xs[2]-(deer_length/2), deer_ys[2]-(deer_width/2)]), deer_length, deer_width,angle = 90-deer_yaws[2], fc='0')

carPred13 = patches.Circle(([xCarPred1s[2],yCarPred1s[2]]),radius, fc ='1')
carPred23 = patches.Circle(([xCarPred2s[2],yCarPred2s[2]]),radius, fc ='1')
carPred33 = patches.Circle(([xCarPred3s[2],yCarPred3s[2]]),radius, fc ='1')
carPred43 = patches.Circle(([xCarPred4s[2],yCarPred4s[2]]),radius, fc ='1')
carPred53 = patches.Circle(([xCarPred5s[2],yCarPred5s[2]]),radius, fc ='1')

deerPred13 = patches.Circle(([xDeerPred1s[2],yDeerPred1s[2]]),radius, fc = '1')
deerPred23 = patches.Circle(([xDeerPred2s[2],yDeerPred2s[2]]),radius, fc = '1')
deerPred33 = patches.Circle(([xDeerPred3s[2],yDeerPred3s[2]]),radius, fc = '1')
deerPred43 = patches.Circle(([xDeerPred4s[2],yDeerPred4s[2]]),radius, fc = '1')
deerPred53 = patches.Circle(([xDeerPred5s[2],yDeerPred5s[2]]),radius, fc = '1')

ax.add_patch(background)
ax.add_patch(road)
ax.add_patch(left_line)
ax.add_patch(right_line)
ax.add_patch(center_line_1)
ax.add_patch(center_line_2)
ax.add_patch(car3)
ax.add_patch(deer3)
ax.add_patch(trees)


ax.add_patch(carPred13)
ax.add_patch(carPred23)
ax.add_patch(carPred33)
ax.add_patch(carPred43)
ax.add_patch(carPred53)

ax.add_patch(deerPred13)
ax.add_patch(deerPred23)
ax.add_patch(deerPred33)
ax.add_patch(deerPred43)
ax.add_patch(deerPred53)


ax = fig.add_subplot(322)
plt.axis('equal')
ax.set_title('$t = t_0 + 0.9s$')
ax.spines['bottom'].set_color('1')
ax.spines['top'].set_color('1')
ax.spines['left'].set_color('1')
ax.spines['right'].set_color('1')
ax.tick_params(axis ='x', colors ='1')
ax.tick_params(axis ='y', colors ='1')
ax.set_xlim(60, 180)

background = patches.Rectangle((-100,-100),10000,10000,facecolor='1')
road = patches.Rectangle((-100,-3.75),10000,11,fc='0.8')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='0')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='0')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='0')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='0')
trees = patches.Rectangle((-100,-100),10000,90,fc='0.8',alpha=0.5)


car4 = patches.Rectangle(([car_xs[3]-(car_length/2), car_ys[3]-(car_width/2)]), car_length, car_width,angle = car_yaws[3], fc='0')
deer4 = patches.Rectangle(([deer_xs[3]-(deer_length/2), deer_ys[3]-(deer_width/2)]), deer_length, deer_width,angle = 90-deer_yaws[3], fc='0')

carPred14 = patches.Circle(([xCarPred1s[3],yCarPred1s[3]]),radius, fc ='1')
carPred24 = patches.Circle(([xCarPred2s[3],yCarPred2s[3]]),radius, fc ='1')
carPred34 = patches.Circle(([xCarPred3s[3],yCarPred3s[3]]),radius, fc ='1')
carPred44 = patches.Circle(([xCarPred4s[3],yCarPred4s[3]]),radius, fc ='1')
carPred54 = patches.Circle(([xCarPred5s[3],yCarPred5s[3]]),radius, fc ='1')

deerPred14 = patches.Circle(([xDeerPred1s[3],yDeerPred1s[3]]),radius, fc = '1')
deerPred24 = patches.Circle(([xDeerPred2s[3],yDeerPred2s[3]]),radius, fc = '1')
deerPred34 = patches.Circle(([xDeerPred3s[3],yDeerPred3s[3]]),radius, fc = '1')
deerPred44 = patches.Circle(([xDeerPred4s[3],yDeerPred4s[3]]),radius, fc = '1')
deerPred54 = patches.Circle(([xDeerPred5s[3],yDeerPred5s[3]]),radius, fc = '1')


ax.add_patch(background)
ax.add_patch(road)
ax.add_patch(left_line)
ax.add_patch(right_line)
ax.add_patch(center_line_1)
ax.add_patch(center_line_2)
ax.add_patch(car4)
ax.add_patch(deer4)
ax.add_patch(trees)


ax.add_patch(carPred14)
ax.add_patch(carPred24)
ax.add_patch(carPred34)
ax.add_patch(carPred44)
ax.add_patch(carPred54)

ax.add_patch(deerPred14)
ax.add_patch(deerPred24)
ax.add_patch(deerPred34)
ax.add_patch(deerPred44)
ax.add_patch(deerPred54)



ax = fig.add_subplot(324)
ax.set_title('$t = t_0 + 1.2s$')
plt.axis('equal')
ax.spines['bottom'].set_color('1')
ax.spines['top'].set_color('1')
ax.spines['left'].set_color('1')
ax.spines['right'].set_color('1')
ax.tick_params(axis ='x', colors ='1')
ax.tick_params(axis ='y', colors ='1')
ax.set_xlim(60, 180)

background = patches.Rectangle((-100,-100),10000,10000,facecolor='1')
road = patches.Rectangle((-100,-3.75),10000,11,fc='0.8')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='0')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='0')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='0')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='0')
trees = patches.Rectangle((-100,-100),10000,90,fc='0.8',alpha=0.5)


car5 = patches.Rectangle(([car_xs[4]-(car_length/2), car_ys[4]-(car_width/2)]), car_length, car_width,angle = car_yaws[4], fc='0')
deer5 = patches.Rectangle(([deer_xs[4]-(deer_length/2), deer_ys[4]-(deer_width/2)]), deer_length, deer_width,angle = 90-deer_yaws[4], fc='0')

carPred15 = patches.Circle(([xCarPred1s[4],yCarPred1s[4]]),radius, fc ='1')
carPred25 = patches.Circle(([xCarPred2s[4],yCarPred2s[4]]),radius, fc ='1')
carPred35 = patches.Circle(([xCarPred3s[4],yCarPred3s[4]]),radius, fc ='1')
carPred45 = patches.Circle(([xCarPred4s[4],yCarPred4s[4]]),radius, fc ='1')
carPred55 = patches.Circle(([xCarPred5s[4],yCarPred5s[4]]),radius, fc ='1')
deerPred15 = patches.Circle(([xDeerPred1s[4],yDeerPred1s[4]]),radius, fc = '1')
deerPred25 = patches.Circle(([xDeerPred2s[4],yDeerPred2s[4]]),radius, fc = '1')
deerPred35 = patches.Circle(([xDeerPred3s[4],yDeerPred3s[4]]),radius, fc = '1')
deerPred45 = patches.Circle(([xDeerPred4s[4],yDeerPred4s[4]]),radius, fc = '1')
deerPred55 = patches.Circle(([xDeerPred5s[4],yDeerPred5s[4]]),radius, fc = '1')

ax.add_patch(background)
ax.add_patch(road)
ax.add_patch(left_line)
ax.add_patch(right_line)
ax.add_patch(center_line_1)
ax.add_patch(center_line_2)
ax.add_patch(car5)
ax.add_patch(deer5)
ax.add_patch(trees)


ax.add_patch(carPred15)
ax.add_patch(carPred25)
ax.add_patch(carPred35)
ax.add_patch(carPred45)
ax.add_patch(carPred55)

ax.add_patch(deerPred15)
ax.add_patch(deerPred25)
ax.add_patch(deerPred35)
ax.add_patch(deerPred45)
ax.add_patch(deerPred55)

ax = fig.add_subplot(326)
ax.set_title('$t = t_0 + 1.5s$')
plt.axis('equal')
ax.spines['bottom'].set_color('1')
ax.spines['top'].set_color('1')
ax.spines['left'].set_color('1')
ax.spines['right'].set_color('1')
ax.tick_params(axis ='x', colors ='1')
ax.tick_params(axis ='y', colors ='1')
ax.set_xlim(60, 180)

background = patches.Rectangle((-100,-100),10000,10000,facecolor='1')
road = patches.Rectangle((-100,-3.75),10000,11,fc='0.8')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='0')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='0')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='0')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='0')
trees = patches.Rectangle((-100,-100),10000,90,fc='0.8',alpha=0.5)


car6 = patches.Rectangle(([car_xs[5]-(car_length/2), car_ys[5]-(car_width/2)]), car_length, car_width,angle = car_yaws[5], fc='0')
deer6 = patches.Rectangle(([deer_xs[5]-(deer_length/2), deer_ys[5]-(deer_width/2)]), deer_length, deer_width,angle = 90-deer_yaws[5], fc='0')

carPred16 = patches.Circle(([xCarPred1s[5],yCarPred1s[5]]),radius, fc ='1')
carPred26 = patches.Circle(([xCarPred2s[5],yCarPred2s[5]]),radius, fc ='1')
carPred36 = patches.Circle(([xCarPred3s[5],yCarPred3s[5]]),radius, fc ='1')
carPred46 = patches.Circle(([xCarPred4s[5],yCarPred4s[5]]),radius, fc ='1')
carPred56 = patches.Circle(([xCarPred5s[5],yCarPred5s[5]]),radius, fc ='1')
deerPred16 = patches.Circle(([xDeerPred1s[5],yDeerPred1s[5]]),radius, fc = '1')
deerPred26 = patches.Circle(([xDeerPred2s[5],yDeerPred2s[5]]),radius, fc = '1')
deerPred36 = patches.Circle(([xDeerPred3s[5],yDeerPred3s[5]]),radius, fc = '1')
deerPred46 = patches.Circle(([xDeerPred4s[5],yDeerPred4s[5]]),radius, fc = '1')
deerPred56 = patches.Circle(([xDeerPred5s[5],yDeerPred5s[5]]),radius, fc = '1')

ax.add_patch(background)
ax.add_patch(road)
ax.add_patch(left_line)
ax.add_patch(right_line)
ax.add_patch(center_line_1)
ax.add_patch(center_line_2)
ax.add_patch(car6)
ax.add_patch(deer6)
ax.add_patch(trees)


ax.add_patch(carPred16)
ax.add_patch(carPred26)
ax.add_patch(carPred36)
ax.add_patch(carPred46)
ax.add_patch(carPred56)

ax.add_patch(deerPred16)
ax.add_patch(deerPred26)
ax.add_patch(deerPred36)
ax.add_patch(deerPred46)
ax.add_patch(deerPred56)

plt.tight_layout(pad=0, w_pad=0, h_pad=0)

plt.savefig('MPC_PredPlot.png',dpi=500)

plt.show()



