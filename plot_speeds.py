from matplotlib import *
from matplotlib.pyplot import *
from numpy import *

distance = [0,20,40,60,80,100]
speeds_nothing = [20,23,20,23,24,20]
speeds_trees = [20,22,12,14,21,20]

figure()
plot(distance, speeds_nothing, 'ko-')
plot(distance, speeds_trees, 'kx-')
xlabel('Starting X-Position (m)')
ylabel('Maximum Safe Speed (m/s)')
legend(['Open Field','Wismer Road'])
ylim(10,30)

show()
