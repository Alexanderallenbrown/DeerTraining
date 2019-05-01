from numpy import *
from matplotlib.pyplot import *

mapa_list = ['nothing','constant_4','constant_7','constant_10']
marker_list = ['x','o','^','s']
color_list = ['k','b']
agent_list = ['Fb','D']

f1 = figure()
a11 = subplot(224)
ylabel('Average Minimum Distance (m)')
xlabel('Speed (m/s)')
a11.set_ylim(0,30)
a12 = subplot(221)
ylabel('Average Minimum Distance (m)')
xlabel('Speed (m/s)')
a12.set_ylim(0,30)
a13 = subplot(223)
ylabel('Average Minimum Distance (m)')
xlabel('Speed (m/s)')
a13.set_ylim(0,30)
a14 = subplot(222)
ylabel('Average Minimum Distance (m)')
xlabel('Speed (m/s)')
a14.set_ylim(0,30)


if len(agent_list)>1:
    legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds','1-$\sigma$ bounds'])
else:
    legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds'])
f2 = figure()
a21 = subplot(224)
ylabel('Collision Probability')
xlabel('Speed (m/s)')
a21.set_ylim(-0.1,1.1)
a22 = subplot(221)
ylabel('Collision Probability')
xlabel('Speed (m/s)')
a22.set_ylim(-0.1,1.1)
a23 = subplot(223)
ylabel('Collision Probability')
xlabel('Speed (m/s)')
a23.set_ylim(-0.1,1.1)
a24 = subplot(222)
ylabel('Collision Probability')
xlabel('Speed (m/s)')
a24.set_ylim(-0.1,1.1)


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

        for setSpeed in range(13,26): 

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


        
        #a1.plot(speedVec,distanceVec,color = color_list[j], marker = marker_list[i])
        # plot(speedVec,std1,'k--')
        # plot(speedVec,std2,'k--')
        #poly = Polygon(sigma1,closed=True,facecolor = '0.9')
        #a1.add_patch(poly)
        #if len(agent_list)>1:
        #    a1.legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds','1-$\sigma$ bounds'])
        #else:
        #    a1.legend(['$y_{sb} = \infty$','$y_{sb} = 4m$', '$y_{sb} = 7m$','$y_{sb} = 10m$','1-$\sigma$ bounds'])


        if mapa == 'nothing':
            a11.plot(speedVec,distanceVec, marker = marker_list[j],color = 'k')
            a11.set_title('$y_{sb} = \infty$')
            poly = Polygon(sigma1,closed=True,facecolor = '0.9')
            a11.add_patch(poly)
            if len(agent_list)>1:
                a11.legend(['MPC','Braking Only','$1-\sigma$ bounds'])
            else:
                if agent_list[0] == 'D':
                    a11.legend(['Braking Only','$1-\sigma$ bounds'])
                else:
                    a11.legend(['MPC','$1-\sigma$ bounds'])

            a21.plot(speedVec,collisionProb, marker = marker_list[j],color = 'k')
            a21.set_title('$y_{sb} = \infty$')
            if len(agent_list)>1:
                a21.legend(['MPC','Braking Only'])
            else:
                if agent_list[0] == 'D':
                    a21.legend(['Braking Only'])
                else:
                    a21.legend(['MPC'])

        if mapa == 'constant_4':
            a12.plot(speedVec,distanceVec, marker = marker_list[j],color = 'k')
            a12.set_title('$y_{sb} = 4m$')
            poly = Polygon(sigma1,closed=True,facecolor = '0.9')
            a12.add_patch(poly)
            if len(agent_list)>1:
                a12.legend(['MPC','Braking Only','$1-\sigma$ bounds'])
            else:
                if agent_list[0] == 'D':
                    a12.legend(['Braking Only','$1-\sigma$ bounds'])
                else:
                    a12.legend(['MPC','$1-\sigma$ bounds'])

            a22.plot(speedVec,collisionProb, marker = marker_list[j],color = 'k')
            a22.set_title('$y_{sb} = 4m$')
            if len(agent_list)>1:
                a22.legend(['MPC','Braking Only'])
            else:
                if agent_list[0] == 'D':
                    a22.legend(['Braking Only'])
                else:
                    a22.legend(['MPC'])

        if mapa == 'constant_7':
            a13.plot(speedVec,distanceVec, marker = marker_list[j],color = 'k')
            a13.set_title('$y_{sb} = 7m$')
            poly = Polygon(sigma1,closed=True,facecolor = '0.9')
            a13.add_patch(poly)
            if len(agent_list)>1:
                a13.legend(['MPC','Braking Only','$1-\sigma$ bounds'])
            else:
                if agent_list[0] == 'D':
                    a13.legend(['Braking Only','$1-\sigma$ bounds'])
                else:
                    a13.legend(['MPC','$1-\sigma$ bounds'])

            a23.plot(speedVec,collisionProb, marker = marker_list[j],color = 'k')
            a23.set_title('$y_{sb} = 7m$')
            if len(agent_list)>1:
                a23.legend(['MPC','Braking Only'])
            else:
                if agent_list[0] == 'D':
                    a23.legend(['Braking Only'])
                else:
                    a23.legend(['MPC'])
        
        if mapa == 'constant_10':
            a14.plot(speedVec,distanceVec, marker = marker_list[j],color = 'k')
            a14.set_title('$y_{sb} = 10m$')
            poly = Polygon(sigma1,closed=True,facecolor = '0.9')
            a14.add_patch(poly)
            if len(agent_list)>1:
                a14.legend(['MPC','Braking Only','$1-\sigma$ bounds'])
            else:
                if agent_list[0] == 'D':
                    a14.legend(['Braking Only','$1-\sigma$ bounds'])
                else:
                    a14.legend(['MPC','$1-\sigma$ bounds'])

            a24.plot(speedVec,collisionProb, marker = marker_list[j],color = 'k')
            a24.set_title('$y_{sb} = 10m$')
            if len(agent_list)>1:
                a24.legend(['MPC','Braking Only'])
            else:
                if agent_list[0] == 'D':
                    a24.legend(['Braking Only'])
                else:
                    a24.legend(['MPC'])





show()

