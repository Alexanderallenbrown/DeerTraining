import sys
from numpy import *
from BicycleModel import *
from matplotlib.pyplot import *
from Driver import *

class Deer:

    def __init__(self, Psi0_Deer = -.307, Sigma_Psi = 0.131, tturn_Deer = 0.594, Vmax_Deer = 13.5, Tau_Deer = 2.203, dT = 1./60, x_Deer = 0, y_Deer = 0):
       
        self.Tau_Deer = Tau_Deer  
        self.Vmax_Deer = Vmax_Deer 
        self.tturn_Deer = tturn_Deer
        self.Sigma_Psi = Sigma_Psi
        self.Psi0_Deer = Psi0_Deer
        self.Psi1_Deer = self.Psi0_Deer + random.randn()*self.Sigma_Psi
        self.Psi2_Deer = self.Psi1_Deer + random.randn()*self.Sigma_Psi # Add the change in angle according to Sigma
        self.dT = dT
        self.x_Deer = x_Deer
        self.y_Deer = y_Deer
        self.xdot_Deer = 0.
        self.ydot_Deer = 0.
        self.x_StartDeer = 60.
        self.Speed_Deer = 0.
        self.Psi_Deer = Psi0_Deer
        self.Amax_Deer = 0.632*Vmax_Deer/Tau_Deer
        self.Psidot_Deer = (self.Psi2_Deer-self.Psi1_Deer)/(10*self.dT)
        if self.Psidot_Deer == 0:
            self.Vturn_Deer = 0
        else:
            self.Vturn_Deer = abs(self.Amax_Deer/self.Psidot_Deer)
        self.tmove_Deer = 0

    def updateDeer(self,x_Car):

        if (self.x_Deer - x_Car) > self.x_StartDeer:
            pass

        else:

            if self.tmove_Deer < self.tturn_Deer:
                self.Psi_Deer = self.Psi1_Deer
                Vstop_Deer = self.Amax_Deer*(self.tturn_Deer-self.tmove_Deer)+self.Vturn_Deer
                

                if (self.Vmax_Deer*(1-exp(-self.tmove_Deer/self.Tau_Deer)) < Vstop_Deer):
                    self.Speed_Deer += self.dT/(self.Tau_Deer)*(self.Vmax_Deer-self.Speed_Deer)

                else:
                    self.Speed_Deer = self.Speed_Deer - self.Amax_Deer*self.dT

            else:
                self.Psi_Deer = self.Psi2_Deer
                self.Speed_Deer += self.dT/(self.Tau_Deer)*(self.Vmax_Deer-self.Speed_Deer)

            self.tmove_Deer += self.dT

        self.xdot_Deer = self.Speed_Deer*sin(self.Psi_Deer)
        self.ydot_Deer = self.Speed_Deer*cos(self.Psi_Deer)

        self.x_Deer += self.dT * self.xdot_Deer
        self.y_Deer += self.dT * self.ydot_Deer

        

        return array([self.Speed_Deer, self.Psi_Deer, self.x_Deer, self.y_Deer])


if __name__=='__main__':

    #set up our deer
    deer = Deer()
    deer.x_Deer = 80
    deer.y_Deer = -2

    simtime = 10
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep

    #now set up the car's parameters
    car = BicycleModel(dT=dt)
    steervec = zeros(len(t))

    #set up the driver
    driver = Driver(dt = dt)
    drive = zeros(3)

    #initialize matrices to hold simulation data
    #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    #now simulate!!
    for k in range(1,len(t)):

        carx_now = carx[k-1,:]
        print carx_now

        drive[:] = driver.driving(carx = carx_now, deer_x = deerx[k-1,2], setSpeed = 20, brake = 'on', yr = -3.5)

        carx[k,:],junk=car.heuns_update(brake = drive[1], gas = drive[0], steer = drive[2], cruise = 'off')
        deerx[k,:] = deer.updateDeer(car.x[2])

    distance = sqrt((carx[:,2]-deerx[:,2])**2+(carx[:,0]-deerx[:,3])**2)

    #now plot stuff
    figure()
    plot(carx[:,2],carx[:,0],'ko',deerx[:,2],deerx[:,3],'ro')

    figure()
    subplot(2,1,1)
    plot(t,deerx[:,2])
    subplot(2,1,2)
    plot(t,deerx[:,3])

    deermoving_index = nonzero((deerx[:,2]-carx[:,2])<=deer.x_StartDeer)
    deermoving_index = deermoving_index[0][0]-1
    turn_index = nonzero(t>=(deer.tturn_Deer+t[deermoving_index]))
    turn_index = turn_index[0][0]

    figure()
    subplot(2,1,1)
    plot(t,deerx[:,0],t[deermoving_index],deerx[deermoving_index,0],'ro',t[turn_index],deerx[turn_index,0],'bo')
    subplot(2,1,2)
    plot(t,carx[:,2],t[deermoving_index],carx[deermoving_index,2],'ro')

    figure()
    plot(t, carx[:,3])

    print(deer.Psi0_Deer,deer.Psi1_Deer,deer.Psi2_Deer)
    print(deer.Vturn_Deer,deer.Vmax_Deer)

    figure()
    plot(t,distance)


    show()