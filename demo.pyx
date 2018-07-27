from numpy import *
import sys
from BicycleModel import *
from Deer import *
from Driver import *

def BinaryConversion(ind):

    resolution = 5

    # Set minimum and maximum values

    Psi0_min = -3.14/2 # radians
    Psi0_max = 3.14/2 # radians

    SigmaPsi_min = 0 # radians
    SigmaPsi_max = 0.45*3.24 # radians

    tturn_min = 0.4 # seconds
    tturn_max = 2.5 # seconds

    Vmax_min = 5 # m/s
    Vmax_max = 18 # m/s

    Tau_min = 0.75 # seconds
    Tau_max = 5 # seconds

    # Divide individual into different binary 
    Psi0_bin = ind[0:resolution]
    SigmaPsi_bin = ind[resolution:2*resolution]
    tturn_bin = ind[2*resolution:3*resolution]
    Vmax_bin = ind[3*resolution:4*resolution]
    Tau_bin = ind[4*resolution:5*resolution]

    # Convert from binary to decimala
    Psi0 = Psi0_min + (Psi0_max - Psi0_min)*float(int(Psi0_bin,2))/((2**resolution)-1)
    SigmaPsi = SigmaPsi_min + (SigmaPsi_max - SigmaPsi_min)*float(int(SigmaPsi_bin,2))/((2**resolution)-1)
    tturn = tturn_min + (tturn_max - tturn_min)*float(int(tturn_bin,2))/((2**resolution)-1)
    Vmax = Vmax_min + (Vmax_max - Vmax_min)*float(int(Vmax_bin,2))/((2**resolution)-1)
    Tau = Tau_min + (Tau_max - Tau_min)*float(int(Tau_bin,2))/((2**resolution)-1)

    #Rrint results
    # print(Psi0)
    # print(SigmaPsi)
    # print(tturn)
    # print(Vmax)
    # print(Tau)

    return array([Psi0,SigmaPsi,tturn,Vmax,Tau])

if __name__=='__main__':

    deer_ind = '1010000001110010110011110'

    agent = "C"

    if agent == "B":
        
        setSpeed = 25
        brake = 'off'
        brakeTime = 0
        yr = 0

    if agent == "C":

        setSpeed = 25
        brake = 'off'
        brakeTime = 0
        yr = 3.5

    if agent == "D":

        setSpeed = 25
        brake = 'on'
        brakeTime = 3
        yr = 0

    if agent == "E":

        setSpeed = 25
        brake = 'on'
        brakeTime = 3
        yr = 3.5


    deer_ind = BinaryConversion(deer_ind)

    deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

    # Indicate deer initial position
    deer.x_Deer = 80
    deer.y_Deer = -2
        
    # Define simulation time and dt
    simtime = 10
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep\

    #now set up the car's parameters        
    car = BicycleModel(dT=dt,U=20)
    steervec = zeros(len(t))

    #set up the driver
    driver = Driver(dt = dt)
    drive = zeros(3)

    #initialize matrices to hold simulation data
    #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    #now simulate!!
    for k in range(1,len(t)):

        carx_now = carx[k-1,:]

        drive[:] = driver.driving(carx = carx_now, deer_x = deerx[k-1,2], setSpeed = setSpeed, brake = brake, yr = yr, brakeTime = brakeTime)

        carx[k,:],carxdot[k,:] = car.heuns_update(brake = drive[1], gas = drive[0], steer = drive[2], cruise = 'off')
        deerx[k,:] = deer.updateDeer(car.x[2])

        
    distance = sqrt((carx[:,2]-deerx[:,2])**2+(carx[:,0]-deerx[:,3])**2)

    #now plot stuff
    figure(1)
    plot(t,carx[:,0])
    xlabel('Time (s)')
    ylabel('Car Y (m)')
    ylim([-5,5])

    figure(2)
    plot(t,carx[:,1])    
    xlabel('Time (s)')
    ylabel('Car V (m/s)')

    figure(3)
    plot(t,carx[:,2])
    xlabel('Time (s)')
    ylabel('Car X (m)')    

    figure(4)
    plot(t,carx[:,3])
    xlabel('Time (s)')
    ylabel('Car U (m/s)')
    ylim([20,30])

    figure(5)
    plot(t,carx[:,4])
    xlabel('Time (s)')
    ylabel('Car Psi (rad)')

    figure(6)
    plot(t,carx[:,5])
    xlabel('Time (s)')
    ylabel('Car R (rad/s)')

    figure(7)
    plot(t,carxdot[:,0])
    xlabel('Time (s)')
    ylabel('Car Y_dot (m/s)')
    ylim([-5,5])

    figure(8)
    plot(t,carxdot[:,1])    
    xlabel('Time (s)')
    ylabel('Car V_dot (m/s^2)')

    figure(9)
    plot(t,carxdot[:,2])
    xlabel('Time (s)')
    ylabel('Car X_dot (m/s)')    

    figure(10)
    plot(t,carxdot[:,3])
    xlabel('Time (s)')
    ylabel('Car U_dot (m/s^2)')
    ylim([20,30])

    figure(11)
    plot(t,carxdot[:,4])
    xlabel('Time (s)')
    ylabel('Car Psi_dot (rad/s)')
    ylim([20,30])

    figure(12)
    plot(t,carxdot[:,5])
    xlabel('Time (s)')
    ylabel('Car r_dot (rad/s^2)')
    ylim([20,30])

    figure(13)
    plot(t,deerx[:,2])
    xlabel('Time (s)')
    ylabel('Deer X (m)')

    figure(13)
    plot(t,deerx[:,3])
    xlabel('Time (s)')
    ylabel('Deer Y (m)')

    figure()
    plot(carx[:,2],carx[:,0],'ko',deerx[:,2],deerx[:,3],'ro')
    xlabel('X')
    ylabel('Y')
    legend(['Car', 'Deer'])


    # figure()
    # subplot(2,1,1)
    # plot(t,deerx[:,2])
    # subplot(2,1,2)
    # plot(t,deerx[:,3])

    # deermoving_index = nonzero((deerx[:,2]-carx[:,2])<=deer.x_StartDeer)
    # deermoving_index = deermoving_index[0][0]-1
    # turn_index = nonzero(t>=(deer.tturn_Deer+t[deermoving_index]))
    # turn_index = turn_index[0][0]

    # figure()
    # subplot(2,1,1)
    # plot(t,deerx[:,0],t[deermoving_index],deerx[deermoving_index,0],'ro',t[turn_index],deerx[turn_index,0],'bo')
    # subplot(2,1,2)
    # plot(t,carx[:,2],t[deermoving_index],carx[deermoving_index,2],'ro')

    # figure()
    # plot(t, carx[:,3])

    # print(deer.Psi0_Deer,deer.Psi1_Deer,deer.Psi2_Deer)
    # print(deer.Vturn_Deer,deer.Vmax_Deer)

    figure()
    plot(t,distance)
    xlabel('Time (s)')
    ylabel('Distance (m)')


    show()


