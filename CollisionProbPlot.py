from numpy import *
from matplotlib.pyplot import *

mapa = 'nothing'
agent = 'D'
xCar = 0
speedVec = []
collisionProb = []
distanceVec = []

for setSpeed in range(19,26): 

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

    print(distanceVec)

    figure()
    hist(IQM)


    # figure()
    # hist(IQM)
    # show()
    # print(collisionProb)

print(speedVec)
print(collisionProb)

figure()
plot(speedVec,collisionProb)

figure()
plot(speedVec,distanceVec)
show()

