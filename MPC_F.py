from numpy import *
from matplotlib.pyplot import *
from rigidfish import RigidFish
from scipy.optimize import minimize
import copy

class MPC_F:
    def __init__(self, Np=60, dtp=1.0/60.0,q_lane_error = 1.0,q_obstacle_error = 1.0,q_steering_effort=1.0,steering_angle_max=.20, self.epsilon = 0.001):
        pass

    def predictDeer_static(self,deernow):
        xdeer = None
        return xdeer

    def predictCar(self,carnow,steervector):
        predictcar = copy.deepcopy(carnow)

        xcar = None
        return xcar

    def ObjectiveFn(self,steervector,carnow,deernow,yroad):
        #Np rows by 6 columns, one for each state (or vice versa)
        xcar_pred = self.predictCar(carnow,steervector)
        xdeer_ pred = sellf.predictDeer_static(deernow)
        #Np rows by 5 columns, one for x and y of deer


        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.
        for k in range(0,self.Np):
            distance = sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2 + (xcar_pred[k,2] - xdeer_pred[k,2])**2)
            J = J +  self.q_steering_effort[k] * (steervector[k])**2 + self.q_lane_error[k] * (xcar_pred[0]-yroad)**2 + self.q_obstacle_error[k] * 1/(distance[k]+self.epsilon)**2
        return J

        def calcOptimal(self,carnow, deernow,yroad):
            steervector = 0.01*random.randn(self.Np)

            bounds = [(-self.steering_angle_max,self.steering_angle_max)]
            for ind in range(1,len(Np)):
                bounds.insert(0,(-self,steering_angle_max,self.steering_angle_max))

            umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method = 'SLSQP')

            opt_steering = umpc.x[0]

            return opt_steering

def demo():

    swerveDistance = 50.0

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


    car = BycicleModel(dT = dt, U = 25.0)

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

            if ((deer.x_Deer - car.x[2]) < swerveDistance): 
                opt_steer = MPC.calcOptim(carx[k-1,:],deerx[k-1,:])
            else
                opt_steer = 0

            carx[k,:],carxdot[k,:] = car.heuns_update(steer = opt_steer, setspeed = 25.0)
            deer[k,:] = deer.updateDeer(car.x[2])

if __name__ = '__main__':







        
