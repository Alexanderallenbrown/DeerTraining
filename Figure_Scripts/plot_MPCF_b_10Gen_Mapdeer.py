from numpy import *
from matplotlib.pyplot import *
import os

dirname = '../GenerationFiles/generationsFb'

gennums = []
mindists = []

for i, filename in enumerate(sorted(os.listdir(dirname))):
    print i,filename
    sgennum = filename[10:-4]
    gennum = int(sgennum)
    thisgen = loadtxt(dirname+'/'+filename)
    mindist = mean(thisgen[:,1])
    gennums.append(gennum)
    mindists.append(mindist)

gennums = array(gennums)
mindists = array(mindists)

inds = argsort(gennums)
gennums = gennums[inds] 
mindists = mindists[inds]

figure()
ax = subplot(111)
ax.plot(gennums,mindists,'k-o')
ax.plot([-1,11],[2,2],'r--')
ax.annotate('collision threshold', xy=(0, 2), xytext=(1, 4),
            arrowprops=dict(facecolor='black', shrink=.05),
            )
xlim([0,11])
xlabel('Generation Number')
ylabel('Mean IQM minimum distance (m)')

savefig('Fb_mapdeer_generation_distance.pdf')
savefig('Fb_mapdeer_generation_distance.png')

show()
