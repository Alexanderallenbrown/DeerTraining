from numpy import *
from matplotlib.pyplot import *

mapa_list = ['nothing','constant_4','constant_7','constant_10']
marker_list = ['o','x','*','.']
fig, ax = subplots()

for i in range(0,len(mapa_list)):
    mapa = mapa_list[i]
    agent = 'D'
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


    
    plot(speedVec,distanceVec,color = 'k', marker = marker_list[i])
    # plot(speedVec,std1,'k--')
    # plot(speedVec,std2,'k--')
    poly = Polygon(sigma1,closed=True,facecolor = '0.9')
    ax.add_patch(poly)
    ylabel('Minimum Average Distance (m)')
    xlabel('Speed (m/s)')
    legend(['Open Field','Constant 4m', 'Constant 7m','Constant 10m','1-$\sigma$ bounds'])

show()

