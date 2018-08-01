from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
import copy

class MPC_F:
    def __init__(self, Np=60, dtp=1.0/60.0,q_lane_error = 1.0,q_obstacle_error = 1.0,q_steering_effort=1.0,steering_angle_max=.20, epsilon = 0.001):
        self.Np = Np
        self.dtp = dtp
        self.q_lane_error = q_lane_error
        self.q_obstacle_error = q_obstacle_error
        self.q_steering_effort = q_steering_effort
        self.steering_angle_max = steering_angle_max
        self.epsilon = epsilon

    def predictDeer_static(self,deernow):
        xdeer = copy.deepcopy(deernow)

        xdeer_pred = zeros(self.Np,4)

        for k in range(0,self.Np):
            xdeer_pred[k,:] = xdeer

        return xdeer_pred

    def predictCar(self,carnow,steervector):
        predictCar = copy.deepcopy(carnow)

        xcar_pred = zeros(self.Np,6)

        car1 = BicycleModel(dT = dt, U = 25.0)

        xcar_pred[0,:] = predictCar

        for k in range(0,self.Np):
            xcar_pred[k,:] = car1.heuns_update(steer = steervector[k], setspeed = 25.0)
        
        return xcar_pred

    def ObjectiveFn(self,steervector,carnow,deernow,yroad):
        #Np rows by 6 columns, one for each state (or vice versa)
        xcar_pred = self.predictCar(carnow,steervector)
        xdeer_pred = self.predictDeer_static(deernow)
        #Np rows by 5 columns, one for x and y of deer


        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.
        for k in range(0,self.Np):
            distance = sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2 + (xcar_pred[k,2] - xdeer_pred[k,2])**2)
            J = J +  self.q_steering_effort * (steervector[k])**2 + self.q_lane_error * (xcar_pred[k,0]-yroad)**2 + self.q_obstacle_error * 1/(distance+self.epsilon)**2
            

        return J

    def calcOptimal(self,carnow, deernow,yroad):
        steervector = 0.01*random.randn(self.Np)

        bounds = [(-self.steering_angle_max,self.steering_angle_max)]
        for ind in range(1,self.Np):
            bounds.insert(0,(-self.steering_angle_max,self.steering_angle_max))

        umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method = 'SLSQP')
        opt_steering = umpc.x[0]

        return opt_steering

def demo():

    swerveDistance = 50.0
    setSpeed = 25.0

    deer_ind = '1010000001110010110011110'

    deer_ind = BinaryConversion(deer_ind)

    deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

    # Indicate deer initial position
    deer.x_Deer = 80
    deer.y_Deer = -2
        
    # Define simulation time and dt
    simtime = 10
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = 25.0)

     #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    MPC = MPC_F()


    #now simulate!!
    for k in range(1,len(t)):

            print carx[k-1,:]
            print deerx[k-1,:]

            if ((deer.x_Deer - car.x[2]) < swerveDistance): 
                opt_steer = MPC.calcOptimal(carnow = carx[k-1,:],deernow = deerx[k-1,:], yroad = 0)
            else:
                opt_steer = 0

            carx[k,:],carxdot[k,:] = car.heuns_update(steer = opt_steer, setspeed = 25.0)
            deerx[k,:] = deer.updateDeer(car.x[2])

            print opt_steer

if __name__ == '__main__':
    demo()







        
