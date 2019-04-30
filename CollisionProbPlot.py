from numpy import *
from matplotlib.pyplot import *

mapa_list = ['nothing','constant_4','constant_7','constant_10']
marker_list = ['x','o','^','s']
color_list = ['k','b']
agent_list = ['Fb','D']

f1 = figure()
a1 = subplot(111)
ylabel('Average Minimum Distance (m)')
xlabel('Speed (m/s)')
if len(agent_list)>1:
    legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds','1-$\sigma$ bounds'])
else:
    legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds'])
f2 = figure()
a2 = subplot(111)
ylabel('Collision Probability')
xlabel('Speed (m/s)')


for j in range(0,len(agent_list)):
    agent = agent_list[j]

    for i in range(0,len(mapa_list)):
        mapa = mapa_list[i]
        #agent = 'Fb'
        xCar = 0
        speedVec = []
        collisionProb = []
        distanceVec = []
        std_vec = []

        for setSpeed in range(15,26): 

            genomeFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/genome.txt'
            resultFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/results.txt'

            trialNum = []
            IQM = []
            collisions = []

            with open(resultFileName, "r") as ins:
                for line in ins:
                    values = line.split()
                    trialNum.append(float(values[0]))
                    IQM.append(float(values[1]))
                    collisions.append(float(values[2]))

            # print(trialNum)
            # print(IQM)
            # print(collisions)


            collisionProb.append(sum(collisions)/len(collisions))
            distanceVec.append(float(mean(IQM)))
            speedVec.append(float(setSpeed))
            std_vec.append(std(IQM))

            print(distanceVec)

            # figure()
            # hist(IQM)


        std1 = array(distanceVec) + array(std_vec)
        std2 = array(distanceVec) - array(std_vec)

        # figure()
        # plot(speedVec,collisionProb,'k')

        sigma1 = zeros([2*len(distanceVec),2])
        for ind in range(0,len(distanceVec)):
            sigma1[ind,0] = speedVec[ind]
            sigma1[ind,1] = std1[ind]
            sigma1[len(distanceVec)+ind-1,0] = speedVec[-ind]
            sigma1[len(distanceVec)+ind-1,1] = std2[-ind]
        sigma1[len(distanceVec)+ind,0] = speedVec[-ind-1]
        sigma1[len(distanceVec)+ind,1] = std2[-ind-1]
        print sigma1 


        
        a1.plot(speedVec,distanceVec,color = color_list[j], marker = marker_list[i])
        # plot(speedVec,std1,'k--')
        # plot(speedVec,std2,'k--')
        poly = Polygon(sigma1,closed=True,facecolor = '0.9')
        a1.add_patch(poly)
        if len(agent_list)>1:
            a1.legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds','1-$\sigma$ bounds'])
        else:
            a1.legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds'])


        
        a2.plot(speedVec,collisionProb,color = color_list[j], marker = marker_list[i])
        if len(agent_list)>1:
            a2.legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds','1-$\sigma$ bounds'])
        else:
            a2.legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds'])





show()

