from numpy import *
from scipy.optimize import minimize
from matplotlib.pyplot import *
from matplotlib import *

#gravitational constant
g = 9.81 #m/s/s

#vehicle parameters
uvmax = 0.6*g #the maximum lateral acceleration for the vehicle.
xdotv = 10 #m/s
xv0 = 0#initial x-position of vehicle
yv0=0 #initial y position of vehicle
ydotv0 = 0# initial y velocity of vehicle
u0 = 0#initial value of vehicle y-acceleration

#obstacle variables
xo = 10 #meters, ahead
yo = 0#put the obstacle right in the road center

#environment variables
ylane_right = -2 #meters
ylane_left = 2 #meters, left lane boundary
W = ylane_left-ylane_right

#prediction variables
Np = 100#predict 10 steps into the future
dtP = 0.01#predict in 0.1 second increments.

#optimizer variables
K1 = .1*ones(Np)#weight on input
K2 = 1*ones(Np)#weight on y vehicle
K3 = zeros(Np);K3[-1] = 1000#weight on terminal y-velocity. Only Penalize TERMINAL!!
K4 = 1*ones(Np)#weight on distance from obstacle.
delta = 0.1 # term added to inverse to avoid division by zero

#simulation variables
dt = 0.01 #seconds, the timestep with which we will update the actual simulation.

class  PointMassVehicle:
    def __init__(self,x=0,y=0,xdot=10,ydot=0,dT=0.01,umax = 5):
        self.x = x#initial x position of vehicle
        self.y = y#initial y position of vehicle
        self.xdot = xdot#vehicle forward velocity
        self.ydot = ydot
        self.umax = umax#maximum lateral acceleration
        self.dT = dT
    def updateStates(self,u):
        self.ydot = self.ydot+u*self.dT #euler update of vehicle velocity (constant acceleration)
        self.x = self.x+self.xdot*self.dT#euler update of vehicle position (constant velocity)
        self.y = self.y+self.ydot*self.dT
        #clamp acceleration at the maximum
        if abs(u)>self.umax:
            u = self.umax*sign(u)
        #update y velocity for next timestep

        return array([self.x,self.xdot,self.y,self.ydot])


def  ObjectiveFn_staticobstacle (uvec,yobstacle,xvehicle,pmvehicle,K1,K2,K3):
    #first reset the predictive model's states to where we are currently. 
    pmvehicle.x,pmvehicle.xdot,pmvehicle.y,pmvehicle.ydot = xvehicle[0],xvehicle[1],xvehicle[2],xvehicle[3]
    J = 0 #initialize the objective to zero
    #now loop through and update J for every timestep in the prediction horizon.
    for ind in range(0,len(uvec)): #compute for each step in the prediction horizon
        #print ind
        xv = pmvehicle.updateStates(uvec[ind])
        J = J + K1[ind]*uvec[ind]**2 + K2[ind] *(xv[2])**2 + K3[ind]*xv[3]**2+K4[ind] * 1/(xv[2]-yobstacle[ind]+delta)**2
    return J



dtP  =  0.10
Np  =  10

dt  =  0.01
simtime  =  1  #second#second


   #set up things that will stay the same over time#set up t 
u0 = 0.10*random.randn(Np)#initialize vehicle acceleration to zeros for each of the 10 prediction timesteps
vehiclepredict = PointMassVehicle(xv0,yv0,xdotv,ydotv0,dtP,uvmax)
y_obs = yo*ones(Np)
xvehicle = array([xv0,xdotv,yv0,ydotv0])
#set up constraint on the optimization
my_constraint =({'type':'ineq','fun': 2})
#set up the bounds on umax. This needs to be a list of tuples (min,max)
bounds = [(-uvmax,uvmax)]
for ind in range(1,len(u0)):
    bounds.insert(0,(-uvmax,uvmax))


#set up time and time-dependent variables
vehiclesimulate = PointMassVehicle(xv0,yv0,xdotv,ydotv0,dt,uvmax) #object to hold simulated vehicle states
xvehicle_store = array([xv0,xdotv,yv0,ydotv0])
u_store = array([0])#store the first value of each optimal input 
t = arange(0,simtime,dt)#time vector
y_obstacle = arange(0,1,dt)#make the obstacle move
for ind in range(1,len(t)):
    #pull out the y-position of the obstacle, make into a constant array    
    yobs_now = y_obstacle[ind]*ones(Np)
    umpc = minimize(ObjectiveFn_staticobstacle,u0,args=(yobs_now,xvehicle,vehiclepredict,K1,K2,K3),bounds=bounds,method='SLSQP',options={'disp': False})
    opt_u = umpc.x#pull out the optimal input
    xvehicle = vehiclesimulate.updateStates(opt_u[0])
    xvehicle_store = vstack((xvehicle_store,xvehicle))
    u_store = append(u_store,opt_u[0])


#now plot the input vector U and the vehicle y-position and y-velocity
figure(figsize=(15,5))
subplot(1,3,1)
plot(t,u_store,'k')
xlabel('Time (s)')
ylabel('optimal y-acceleration of vehicle (m/s/s)')
subplot(1,3,2)
plot(t,xvehicle_store[:,2],'k')
xlabel('Time (s)')
ylabel('Optimal Y-position of Vehicle (m)')
subplot(1,3,3)
plot(t,xvehicle_store[:,3],'k')
xlabel('Time (s)')
ylabel('Optimal Y-velocity of vehicle (m/s)')

figure(figsize=(15,5))
plot(xvehicle_store[:,0],xvehicle_store[:,2],'k',xo*ones(len(t)),y_obstacle,'r.',[0, max(xvehicle_store[:,0])],[ylane_left,ylane_left],'r-.',[0, max(xvehicle_store[:,0])],[ylane_right,ylane_right],'r-.')
ylim([-1.1*W,1.1*W])
xlim([1.1*min(xvehicle_store[:,0]),1.1*max(xvehicle_store[:,0])])
legend(['vehicle path','obstacle','lane boundaries'])