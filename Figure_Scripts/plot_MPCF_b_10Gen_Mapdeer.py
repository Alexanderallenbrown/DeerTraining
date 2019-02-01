from numpy import *
from matplotlib.pyplot import *
import os

figure()
legendstr = []

xdir = '../GenerationFiles/FbData/x0_0'


for i, filename in enumerate(sorted(os.listdir(xdir))):

    dirname = '../GenerationFiles/FbData/x0_0/' + filename

    legendstr.append('v = ' + filename[-2:] + ' m/s')

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

    ax = subplot(111)
    ax.plot(gennums,mindists,'-o')


    xlim([0,20])
    xlabel('Generation Number')
    ylabel('Average minimum distance (m)')

ax.plot([-1,21],[2,2],'r--')

legendstr.append('Collision threshold')
legend(legendstr)

savefig('Fb_mapdeer_generation_distance.pdf')
savefig('Fb_mapdeer_generation_distance.png')

show()
