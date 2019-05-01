from matplotlib import *
from matplotlib.pyplot import *
from numpy import *

distance = [0,20,40,60,80,100,120]
speeds_nothing = [22,24,21,23,22,23,21]
speeds_trees = [24,21,14,15,12,17,23]

figure()
plot(distance, speeds_nothing, 'ko-')
plot(distance, speeds_trees, 'kx-')
xlabel('$x_{v0}$ (m)')
ylabel('Maximum Safe Speed (m/s)')
legend(['No cover','Rectangular patch of cover'])
ylim(10,28)

show()
