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

    # nothing starting at 7m/s
    #deer_list = ['0011111111001101111100000','0010111111001101111100000','0010011101001101111000000','0010011010000000111000000','0001111110000111111100000','0000111111000011111100000',
    # starting at 13 m/s
    #deer_list = ['0100011101000001001100000','0011011110000001111000000','0001001001000001110000000','0011010100000001001000000','0100011000000001101100000','0010101010000001111000010','0100011000000001111100100','0001100110000011011000000','0010101000000001111100001','0010100010000000111001000','0100001010000001001100011','0000000000000000001000000','0010000000000000100100000']
    # constant_4 starting at 14m/s
    #deer_list = ['0101101111000001010010011','0100001010000001110010010','1010010100000001011111001','0010100100000001101001001','0111101111000000100100100','1000001010000011010110111','1001110111000001111101110','0100000101000011001010000','1000000100000000000000100','1000000100000111011110100','1001000010000000011110100','0010100000000000001000001','1000010000000001011000011']
    # constant_7 starting at 13m/s
    deer_list = ['0101110000000101111010111','0110001010000000110011110','0101101011000101111111111','0011001101000001111000000','1000010001000001001110000','0011100110000001111010010','0110101111000001101001111','1000001110000001000001001','0100000101000001110111010','0101101000000011000101001','1010001000000001010001111','0001000100000001000000000','1000101111000011110100110']
    # constant_10 starting at 13m/s
    #deer_list = ['0101110010000001101111111','1001011011000001111111111','0100001110000101110110111','0101111000000011100100101','1000010000000001001111011','0111010010000011100101010','1000010100000001111110001','0111101100000010111101100','1000001100000001101110111','0110001001000001000110010','0010000000000000011000001','0010000000000000010000010','0100000000000000000001000']

    start_speed = 26 - len(deer_list)
    print('start speed is ' + str(start_speed) + ' m/s')

    mapa = 'constant_10'

    agent = 'D'

    xCar = 0

    for setSpeed in range(start_speed,15):

        agent_type = agent

        print('The agent is ' + agent)

        n = 100

        deer_ind = deer_list[setSpeed-start_speed]

        CurrentDeer = BinaryConversion(str(deer_ind))

        print('The current deer genome is ' + str(deer_ind))

        DirTrials = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/trialData'

        if not os.path.exists(DirTrials):
            os.makedirs(DirTrials)

        trial_number,min_distance,collision = TestDeer_MPC(CurrentDeer, n, agent, xCar, setSpeed,mapa,DirTrials,int(deer_ind))


        genomeFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/genome.txt'
            
        genomeFile = open(genomeFileName,'w+');
        genomeFile.close();
        genomeFile = open(genomeFileName, 'a');
        genomeFile.write(str(deer_ind))
        genomeFile.close()   


        newFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/results.txt'

        newFile = open(newFileName,'w+');
        newFile.close();
        newFile = open(newFileName, 'a');

        for x in range(0,len(trial_number)):
            newFile.write(str(trial_number[x]) + ' ' + str(min_distance[x]) + ' ' + str(collision[x]))
            newFile.write('\n')
        newFile.close()   


def BinaryConversion(ind):

    resolution = 5

    # Set minimum and maximum values

    Psi1_min = -3.14/2 # radians
    Psi1_max = 3.14/2 # radians

    yinit_min = -4. # meters
    yinit_max = -30. # meters

    dturn_min = 20 # meters
    dturn_max = 120 # meters

    Vmax_min = 5 # m/s
    Vmax_max = 18 # m/s

    Tau_min = 0.75 # seconds
    Tau_max = 5 # seconds

    # Divide individual into different binary 
    Psi1_bin = ind[0:resolution]
    yinit_bin = ind[resolution:2*resolution]
    dturn_bin = ind[2*resolution:3*resolution]
    Vmax_bin = ind[3*resolution:4*resolution]
    Tau_bin = ind[4*resolution:5*resolution]

    # Convert from binary to decimala
    Psi1 = Psi1_min + (Psi1_max - Psi1_min)*float(int(Psi1_bin,2))/((2**resolution)-1)
    yinit = yinit_min + (yinit_max - yinit_min)*float(int(yinit_bin,2))/((2**resolution)-1)
    dturn = dturn_min + (dturn_max - dturn_min)*float(int(dturn_bin,2))/((2**resolution)-1)
    Vmax = Vmax_min + (Vmax_max - Vmax_min)*float(int(Vmax_bin,2))/((2**resolution)-1)
    Tau = Tau_min + (Tau_max - Tau_min)*float(int(Tau_bin,2))/((2**resolution)-1)

    #Rrint results
    # print(Psi1)
    # print(yinit)
    # print(dturn)
    # print(Vmax)
    # print(Tau)

    return array([Psi1,yinit,dturn,Vmax,Tau])

def TestDeer_MPC(deer_ind, n, agent, xCar, setSpeed,fake_map,Dir,deerID):

    min_distance = zeros(n)
    Collision = zeros(n)
    trial_number = zeros(n)

    for k_1 in range(0,n):

        trial_number[k_1] = int(k_1 + 1)

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

        deer = Deer_Escape_Smooth(Psi1_Deer = deer_ind[0], y_init = deer_ind[1], dturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

        # Indicate deer initial position
        deer.x_Deer = x_deer
        # Define simulation time and dt
        simtime = 10
        dt = deer.dT
        t = arange(0,simtime,dt) #takes min, max, and timestep\

        #now set up the car's parameters        
        car = BicycleModel(dT=dt,U=20)
        
        carx = zeros((len(t),len(car.x)))
        carxdot = zeros((len(t),len(car.x)))
        car.x[3] = setSpeed
        car.x[2] = x_car
        carx[0,:] = car.x

        actual_steervec = zeros(len(t))
        command_steervec = zeros(len(t))

        #initialize for deer as well
        deerx = zeros((len(t),4))
        #fill in initial conditions because they're nonzero
        deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])
        distancevec = zeros(len(t))

        weight = 10.0
        swerveDistance = 50.0
        last_steer_t = -0.1
        deerSight = False
        
        if agent == 'Fb':
            MPC = MPC_Fb(q_lane_error = weight,q_obstacle_error =0.0/weight*10,q_lateral_velocity=1.0,q_steering_effort=1.0,q_accel = 0.005,predictionmethod='CV')
            predDirName = Dir + '/preds/ID_' + str(int(deerID))
            predFileName = predDirName + '/trial_' + str(k_1+1) + '.txt'
            if not os.path.exists(predDirName):
                os.makedirs(predDirName)



        # Initialize data saving files
        FileName = Dir + '/trial_' + str(k_1+1) + '.txt'
        
        


        newFile = open(FileName,'w+');
        newFile.close();
        newFile = open(FileName, 'a');

        if agent == 'Fb':
            predF = open(predFileName,'a')


        for k in range(1,len(t)):

            if car.x[3] > 1.0:

                if deerSight == False:

                    distance_pred = zeros(10)

                    distanceAngle = MapRaycasting([car.x[2],car.x[0]],'mapa',KML = KML, fake = fake_map)

                    deerAngle = arctan2((deer.y_Deer-car.x[0]),(deer.x_Deer-car.x[2]))
                    deerDist = sqrt((deer.y_Deer-car.x[0])**2+(deer.x_Deer-car.x[2])**2)

                    deerAngle = int(deerAngle *180./3.1415)

                    if deerAngle < 0:
                        deerAngle = 360 + deerAngle

                    if (deerDist < distanceAngle[deerAngle]): 
                        deerSight = True

                if deerSight == True:

                    if agent == 'Fb':

                        if ((t[k]- last_steer_t) >= MPC.dtp):
                            opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                            brake = MPC.calcBraking(carnow = car)
                            gas = 0

                            last_steer_t = t[k]
        

                    else:
                        opt_steer = 0
                        brake = 0.6
                        gas = 0

                else:
                    opt_steer = 0
                    gas = 0
                    brake = 0

                carx[k,:],carxdot[k,:],actual_steervec[k] = car.rk_update(gas = gas, brake = brake, steer = opt_steer, cruise = 'off')
                deerx[k,:] = deer.updateDeer(car.x[2],car.x[0])
                distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)
                #print carx[k,:]
                #print deerx[k,:]
                #print distancevec[k]

            else:
                carx[k,:] = array([carx[k-1,0],0.0,carx[k-1,2],0.0,carx[k-1,4],0.0])
                carxdot[k,:] = zeros(6)
                actual_steervec[k] = actual_steervec[k-1]
                deerx[k,:] = array([0.0,deerx[k-1,1],deerx[k-1,2],deerx[k-1,3]])
                distancevec[k] = distancevec[k-1]

            command_steervec[k] = opt_steer

            newFile.write(str(t[k])+'\t')
            newFile.write(str(command_steervec[k])+'\t')
            newFile.write(str(actual_steervec[k])+'\t')
            for ind2 in range(0,6):
                newFile.write(str(carx[k,ind2]) + ' \t');
            for ind2 in range(0,6):
                newFile.write(str(carxdot[k,ind2]) + ' \t');
            for ind2 in range(0,4):
                    newFile.write(str(deerx[k,ind2]) + '\t');
            newFile.write('\n')

            if agent == 'Fb':
                predF.write(str(t[k])+'\t')
                for ind2 in range(0,len(MPC.XYPrediction)):  
                    predF.write(str(MPC.XYPrediction[ind2])+'\t')
                for ind2 in range(0,len(MPC.XYDeerPrediction)):
                    predF.write(str(MPC.XYDeerPrediction[ind2])+'\t')
                predF.write('\n')


        distancevec = distancevec[1:len(distancevec)]
        min_distance[k_1] = min(distancevec)

        Collision[k_1] = bool(0)

        # If the minimum distance is under 2m, check for a collision to occue
        if min_distance[k_1] < 2.0:

            check = CollisionCheck()
            Collision[k_1] = check.collision(carx[:,2],carx[:,0],carx[:,4],deerx[:,2],deerx[:,3],deerx[:,1])
            if Collision[k_1] == True:
                Collision[k_1] = bool(1)
            else:
                Collision[k_1] = bool(0)

        newFile.close();
        
        if agent == 'Fb':
            predF.close()

    return trial_number, min_distance, Collision



if __name__=='__main__':
    demo()

