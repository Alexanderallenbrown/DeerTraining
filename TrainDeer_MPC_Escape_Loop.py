from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
import operator;
from TraitResultObject import TraitResult;
from GenDevelopment import *;
import sys
import time
import copy
from MPC_F_braking_KinCar_Map import *
from Deer_Escape import *
from Crash_Test import CollisionCheck
import os



def demo():

    mapa = 'real_tree_wismer'
   
    xCar = 0
    setSpeed = 25

    agent_type = "Fb"

    generation_number = 1
    meanRes = 100. 

    intermediatePopulationSize = 10;
    numberOfHumans = 8
    populationSize = 15;

    n = populationSize
    m = intermediatePopulationSize;
    h = numberOfHumans;

    if generation_number == 1:
        FirstGen(setSpeed,xCar,mapa,h)  

    while True:

        while ((meanRes > 2.0) and (generation_number < 21)):

            print "New generation"

            print generation_number

            generation = generation_number
            agent = agent_type
            #Select agent:
            # A = Human
            # B = Straight
            # C = Swerve
            # D = Brake
            # E = Hybrid

            print generation

            Gfname = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa) + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/Generation' + str(generation) + '.txt';

            print Gfname

            #should have an array of size m*h (of object values )

            #in some way, read in a text file to fill an array
            with open(Gfname, "r") as ins:
                CurrentGenarray = []
                for line in ins:
                    values = line.split()
                    deer = TraitResult();
                    minDistanceVec = values[2:(2+h)]
                    collisionVec = values[(2+h):]
                    deer.assign(str(values[0]),float(values[1]),minDistanceVec,collisionVec);
                    CurrentGenarray.append(deer)

                    print minDistanceVec


            # we now have an arrary of deer objects, paired values of attributes and the corresponding results
            CurrentGenarray.sort(key=operator.attrgetter("result"))

            print "This is the CurrentGenarray"

            print CurrentGenarray

            for x in range(0, len(CurrentGenarray)):
                print CurrentGenarray[x].traits

            for x in range(0, len(CurrentGenarray)):
                print CurrentGenarray[x].result

            NewInterGenArray = []


            print "This is the CurrentGenarray again"

            print CurrentGenarray

            for x in range(0, m):
                develMethod = random.randint(6);
                if develMethod == 0:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do single point cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = SinglePoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 1:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do double point cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = DoublePoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 2:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do random point cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = RandomPoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 3:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do and cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = AndCross(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 4:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do or cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = OrCross(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 5:
                    inds = random.choice(len(CurrentGenarray),1);
                    print str(x) + ' do mutation with ' + str(inds[0]);
                    NewDeer = Mutate(CurrentGenarray[inds[0]]);
                NewInterGenArray.append(NewDeer);

            print '';
            for x in range(0, len(CurrentGenarray)):
                print  str(x) + ' ' + str(CurrentGenarray[x].traits) + ' ' + str(CurrentGenarray[x].result);    
            print '';
            for x in range(0, len(NewInterGenArray)):
                print  str(x) + ' ' + str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            #Test deer in intermediate generation
            for index in range(0,m):
                print "Training deer " + str(index + 1) + " out of " + str(m)
                CurrentDeer = BinaryConversion(str(NewInterGenArray[index].traits))
                print CurrentDeer
                NewInterGenArray[index].result,NewInterGenArray[index].minDistanceVec, NewInterGenArray[index].collisionVec = TestDeer_MPC(CurrentDeer, h, agent, xCar, setSpeed,mapa)
                print NewInterGenArray[index].result
                print NewInterGenArray[index].minDistanceVec
                print NewInterGenArray[index].collisionVec

            for x in range(0, n):
                NewInterGenArray.append(CurrentGenarray[x])

            for x in range(0, len(NewInterGenArray)):
                print str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            #Now, total array of intermediate and base generation, with scores

            NewInterGenArray.sort(key=operator.attrgetter("result"))

            for x in range(0, len(NewInterGenArray)):
                print str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            NewBaseGenArray = []

            for x in range(0, n/2):
                NewBaseGenArray.append(NewInterGenArray[0]);
                NewInterGenArray.pop(0)

            for x in range(0,(n+m)/5):
                NewInterGenArray.pop(len(NewInterGenArray)-1)

            for x in range(0, n/2+1):
                randIndex = random.randint(len(NewInterGenArray))
                NewBaseGenArray.append(NewInterGenArray[randIndex]);
                NewInterGenArray.pop(randIndex);


            for x in range(0, len(NewInterGenArray)):
                print str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            for x in range(0, len(NewBaseGenArray)):
                print str(NewBaseGenArray[x].traits) + ' ' + str(NewBaseGenArray[x].result);    
            print '';

            G2fname = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa)  + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/Generation' + str(generation+1) + '.txt';

            newGenFile = open(G2fname,'w+');
            newGenFile.close();
            newGenFile = open(G2fname, 'a');
            for x in range(0, len(NewBaseGenArray)):
                newGenFile.write(str(NewBaseGenArray[x].traits) + ' ' + str(NewBaseGenArray[x].result))
                print NewBaseGenArray[x].minDistanceVec
                print NewBaseGenArray[x].collisionVec
                for ind in range(0,h):
                    newGenFile.write(' ' + str(NewBaseGenArray[x].minDistanceVec[ind]))
                for ind in range(0,h):
                    newGenFile.write(' ' + str(NewBaseGenArray[x].collisionVec[ind]))
                newGenFile.write('\n')
            newGenFile.close()

            # sumRes = 0.
            # for x in range(0, len(NewBaseGenArray)):
            #     sumRes = sumRes + NewBaseGenArray[x].result
            # meanRes = sumRes/len(NewBaseGenArray)

            meanRes = NewBaseGenArray[0].result

            time.sleep(5)

            generation_number = generation_number + 1

        if meanRes <= 2.0:
            print "Crash reduce SPEED"
            print meanRes
            setSpeed = setSpeed - 1
            meanRes = 1000.0
            generation_number = 1
            FirstGen(setSpeed,xCar,mapa,h)

        else:
            print "We're good move FORWARD"
            print meanRes
            setSpeed = 25
            xCar = xCar + 20
            meanRes = 10.0
            generation_number = 1
            FirstGen(setSpeed,xCar,mapa,h)


def BinaryConversion(ind):

    resolution = 5

    # Set minimum and maximum values

    Psi1_min = -3.14/2 # radians
    Psi1_max = 3.14/2 # radians

    yinit_min = -1. # meters
    yinit_max = -30. # meters

    tturn_min = 0.4 # seconds
    tturn_max = 2.5 # seconds

    Vmax_min = 5 # m/s
    Vmax_max = 18 # m/s

    Tau_min = 0.75 # seconds
    Tau_max = 5 # seconds

    # Divide individual into different binary 
    Psi1_bin = ind[0:resolution]
    yinit_bin = ind[resolution:2*resolution]
    tturn_bin = ind[2*resolution:3*resolution]
    Vmax_bin = ind[3*resolution:4*resolution]
    Tau_bin = ind[4*resolution:5*resolution]

    # Convert from binary to decimala
    Psi1 = Psi1_min + (Psi1_max - Psi1_min)*float(int(Psi1_bin,2))/((2**resolution)-1)
    yinit = yinit_min + (yinit_max - yinit_min)*float(int(yinit_bin,2))/((2**resolution)-1)
    tturn = tturn_min + (tturn_max - tturn_min)*float(int(tturn_bin,2))/((2**resolution)-1)
    Vmax = Vmax_min + (Vmax_max - Vmax_min)*float(int(Vmax_bin,2))/((2**resolution)-1)
    Tau = Tau_min + (Tau_max - Tau_min)*float(int(Tau_bin,2))/((2**resolution)-1)

    #Rrint results
    # print(Psi1)
    # print(yinit)
    # print(tturn)
    # print(Vmax)
    # print(Tau)

    return array([Psi1,yinit,tturn,Vmax,Tau])

def TestDeer_MPC(deer_ind, n, agent, xCar, setSpeed,fake_map):

    min_distance = zeros(n)
    Collision = zeros(n)

    for k_1 in range(0,n):

        MPCDistance = 50.0
        setSpeed = setSpeed
        x_car = xCar
        x_deer = xCar + 80.0
        KML = False
        fake_map = fake_map


        print("Run " + str(k_1+1) + " of " + str(n))
        print("The current car speed is " + str(setSpeed) + " m/s, and the starting x is " + str(xCar) + "m")
        print('KML = ' + str(KML) + ', map = ' + str(fake_map))


        # Where n is the number of drivers we are goin to test each deer against

        deer = Deer_Escape(Psi1_Deer = deer_ind[0], y_init = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

        # Indicate deer initial position
        deer.x_Deer = x_deer
        # Define simulation time and dt
        simtime = 10
        dt = deer.dT
        t = arange(0,simtime,dt) #takes min, max, and timestep\

        #now set up the car's parameters        
        car = BicycleModel(dT=dt,U=20)
        
        carx = zeros((len(t),len(car.x)))
        car.x[3] = setSpeed
        car.x[2] = x_car
        carx[0,:] = car.x

        #initialize for deer as well
        deerx = zeros((len(t),4))
        #fill in initial conditions because they're nonzero
        deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])
        distancevec = zeros(len(t))

        weight = 10.0
        swerveDistance = 50.0
        last_steer_t = 0.
        deerSight = False
        
        MPC = MPC_Fb(q_lane_error = weight,q_obstacle_error =0.0/weight*10,q_lateral_velocity=1.0,q_steering_effort=1.0,q_accel = 0.005,predictionmethod='CV')

        for k in range(1,len(t)):

            distance_pred = zeros(10)

            distanceAngle = MapRaycasting([car.x[2],car.x[0]],'mapa',KML = KML, fake = fake_map)

            deerAngle = arctan((deer.y_Deer-car.x[0])/(deer.x_Deer-car.x[2]))
            deerDist = sqrt((deer.y_Deer-car.x[0])**2+(deer.x_Deer-car.x[2])**2)

            deerAngle = int(deerAngle *180./3.1415)

            if (deerDist < distanceAngle[deerAngle]): 
                deerSight = True

            if deerSight == True:

                if ((t[k]- last_steer_t) >= MPC.dtp):
                    opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                    brake = MPC.calcBraking(carnow = car)
                    gas = 0

                    last_steer_t = t[k]
    
            else:
                opt_steer = 0
                gas = 0
                brake = 0

            carx[k,:],junk1,junk2 = car.rk_update(gas = gas, brake = brake, steer = opt_steer, cruise = 'off')
            deerx[k,:] = deer.updateDeer(car.x[2],car.x[0])
            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)
            #print carx[k,:]
            #print deerx[k,:]
            #print distancevec[k]

        distancevec = distancevec[1:len(distancevec)]
        min_distance[k_1] = min(distancevec)

    Collision[k_1] = bool(0)

    if min_distance[k_1] < 2.0:

        check = CollisionCheck()
        Collision[k_1] = check.collision(carx[:,2],carx[:,0],carx[:,4],deerx[:,2],deerx[:,3],deerx[:,1])
        if Collision[k_1] == True:
            Collision[k_1] = bool(1)
        else:
            Collision[k_1] = bool(0)

    # Calculate IQM

    # Sort values from smallest to largest
    min_distance = sorted(min_distance)
    minDistanceVec = min_distance
    # Eliminate lower and upper quartiles
    min_distance = min_distance[int(round(n/4.0)):int(ceil(3.0*n/4.0))]
    # Calculate the IQM
    avg_min_distance = mean(min_distance)
    # print(avg_min_distance)

    return avg_min_distance, minDistanceVec, Collision


def FirstGen(setSpeed, xCar,mapa,h):

    Deer10 = ['1011110011010101111100000','1000011110110111001101000','0011010011101011111001101','1011001010011011110100111','1110001110010110110101000','0101011010101111100110101','1001011110101011000101110','1110100000110001010111001','1011101101011011001011011','0010100010001101001001111','0101111110001101001100001','1001010110101111010110110','0010010000000111000101001','0001000001001100100110000','0101010000110001110001001']
    #Deer10 = ['0000100000011111111100000','0000000000110111001101000','0000000000001011111001101','0000000000011011110100111','0000100000010110110101000','0000100000101111100110101','0000100000101011000101110','0000100000110001010111001','1011101101011011001011011','0010100010001101001001111','0101111110001101001100001','1001010110101111010110110','0010010000000111000101001','0001000001001100100110000','0101010000110001110001001']

    agent = 'Fb'

    directory = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa) + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) 

    if not os.path.exists(directory):
        os.makedirs(directory)

    Gfname = directory + '/Generation1.txt';

    newGenFile = open(Gfname,'w+');
    newGenFile.close();

    for ind in range(0,len(Deer10)):

        Deer1 = Deer10[(ind)]
        Deer1_bin = Deer1

        print "THIS IS THE DEER WE ARE CURRENTLY TRAINING"
        print str(Deer1)

        Deer1 = BinaryConversion(Deer1)

        print(Deer1)

        Distance1, minDistanceVec,collision = TestDeer_MPC(deer_ind=Deer1, n=h, agent = agent, setSpeed = setSpeed, xCar = xCar, fake_map = mapa)
        print minDistanceVec

        newGenFile = open(Gfname, 'a');
        newGenFile.write(str(Deer1_bin) + ' ' + str(Distance1));
        for ind in range(0,h):
            newGenFile.write(' ' + str(minDistanceVec[ind]))
        for ind in range(0,h):
            newGenFile.write(' ' + str(collision[ind]))
        newGenFile.write('\n')

        newGenFile.close()


        print "MINIMUM DISTANCE"
        print str(Deer1_bin)
        print(Distance1)




if __name__=='__main__':
    demo()

