from matplotlib import *
from matplotlib.pyplot import *
from numpy import *
import matplotlib.patches as patches


distance = [0,20,40,60,80,100,120,140,160,180]
speeds_nothing = [22,24,21,23,22,23,21,22,21,22]
speeds_trees_4 = [24,21,20,15,12,17,21,22,21,20]
speeds_trees_7 = [21,24,18,19,20,18,24,22,23,25]
speeds_trees_10 =[23,25,20,21,22,22,22,22,24,22]



fig = figure()
ax = fig.add_subplot(111)
ax.plot(distance, speeds_nothing, 'ko-')
ax.plot(distance, speeds_trees_4, 'kx-')
ax.plot(distance, speeds_trees_7, 'ks-')
ax.plot(distance, speeds_trees_10, 'k^-')
poly1 = patches.Rectangle([20,0],60,100,fc='0.8',alpha=0.5)
poly2 = patches.Rectangle([80,0],20,100,fc='0.6',alpha=0.5)
poly3 = patches.Rectangle([100,0],60,100,fc='0.8',alpha=0.5)
plot([20,20],[0,100],'k--')
plot([80,80],[0,100],'k--')
plot([100,100],[0,100],'k--')
plot([160,160],[0,100],'k--')


ax.add_patch(poly1)
ax.add_patch(poly2)
ax.add_patch(poly3)

xlabel('$x_{v0}$ (m)',fontsize = 16)
ylabel('Maximum Safe Speed (m/s)', fontsize = 16)
legend(['$y_{sb} = \infty$','$y_{sb} = 4m$','$y_{sb} = 7m$','$y_{sb} = 10m$'], fontsize =14)
ylim(10,28)

savefig('MPC_MaxSpeeds',dpi=600)

show()
