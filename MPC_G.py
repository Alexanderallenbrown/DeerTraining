from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
import copy

class MPC_G:
    def __init__(self, Np=20, dtp=.2,q_obstacle_error = 1.0,q_cruise_speed = 1.0,q_x_accel=1.0, gas_max=250, brake_max = 500, epsilon = 0.0001):
        self.Np = Np
        self.dtp = dtp
        self.q_obstacle_error = q_obstacle_error
        self.q_cruise_speed = q_cruise_speed
        self.q_x_accel = q_x_accel
        self.gas_max = gas_max
        self.brake_max = brake_max
        self.epsilon = epsilon
        self.prediction_time = self.dtp*self.Np
        self.t_horizon = arange(0,self.prediction_time,self.dtp)#go to one extra so the horizon matches

    def predictDeer_static(self,deernow,carnow):
        predictDeer = copy.deepcopy(deernow)
        #make a time vector for prediction using 'fine' timestep of deer
        tvec = arange(0,self.prediction_time+predictDeer.dT,predictDeer.dT)
        #initialize the 'fine' predicted state vector
        xdeer_pred = zeros((len(tvec),4))
        for k in range(0,len(tvec)):
            #eventually, the deer will need to also have a model of how the CAR moves...
            xdeer_pred[k,:] = predictDeer.x_Deer
        #now downsample the prediction so that the horizon matches MPC rather than the deer
        #this way, the MPC will only look at and attempt to optimize a few points, but the prediction will be high-fi
        xdeer_pred_downsampled = zeros((self.Np,4))
        for k in range(0,4):
            xdeer_pred_downsampled[:,k] = interp(self.t_horizon,tvec,xdeer_pred[:,k])
        return xdeer_pred_downsampled

    def predictCar(self,carnow,x_accelvector):
        predictCar = copy.deepcopy(carnow)
        predictCar.tiretype='linear'
        #compute the numper of timesteps we will simulate using the CAR's dt
        timesteps = self.prediction_time/predictCar.dT
        #compute a time vector for predicting
        tvec = arange(0,self.prediction_time+predictCar.dT,predictCar.dT)
        #initialize the downsampled vector we will return
        xcar_pred_downsampled = zeros((self.Np,6))
        xdotcar_pred_downsampled = zeros((self.Np,6))
        #initialize the 'fine' vector we will fill while predicting 
        xcar_pred = zeros((len(tvec),6))
        xdotcar_pred  = zeros((len(tvec),6))
        #we have to 'upsample' the steer vector since it is only Np long. it will look like 'stairs'
        x_accelvector_unsampled = interp(tvec,self.t_horizon,x_accelvector)
        #print steervector_upsampled.shape
        #actually predict the car's states given the input
        for k in range(0,self.Np):
            if x_accelvector_upsampled[k] > 0:
                gas = x_accelvector_upsampled[k]*self.gas_max
                brake = 0
                xcar_pred[k,:],xdotcar_pred[k,:] = predictCar.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
            else:
                gas = 0
                brake = x_accelvector_upsampled[k]*self.brake_max
                xcar_pred[k,:],xdotcar_pred[k,:] = predictCar.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
        #now downsample the prediction so it is only MPC.Np points long
        for k in range(0,6):
            xcar_pred_downsampled[:,k] = interp(self.t_horizon,tvec,xcar_pred[:,k])
            xdotcar_pred_downsampled[:,k] = interp(self.t_horizon,tvec,xdotcar_pred[:,k])
        #print xdotcar_pred_downsampled.shape,xcar_pred_downsampled.shape
        return xcar_pred_downsampled,xdotcar_pred_downsampled

    def ObjectiveFn(self,x_accelvector,carnow,deernow,setSpeed):
        J=0
        #Np rows by 6 columns, one for each state (or vice versa)
        xcar_pred,xdotcar_pred = self.predictCar(carnow,x_accelvector)
        xdeer_pred = self.predictDeer_static(deernow,carnow)

        #Np rows by 5 columns, one for x and y of deer
        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.
        for k in range(0,self.Np):
            distance = sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2+ (xcar_pred[k,2] - xdeer_pred[k,2])**2)
            return distance
            if(carnow.x[2]<deernow.x_Deer):
                J = J + self.q_x_accel * (x_accelvector[k])**2 + self.q_cruise_speed * (xcar_pred[k,3]-setSpeed)**2 + self.q_obstacle_error * (1/(distance+self.epsilon))**2
            else:
                #print "passed deer!"
                J = J + self.q_cruise_speed * (xcar_pred[k,3]-setSpeed)**2
        #return J


    def calcOptimal(self,carnow,deernow,setSpeed):
        x_accelvector = 0.5*random.randn(self.Np)

        bounds = [(-1,1)]
        for ind in range(1,self.Np):
            bounds.insert(0,(-1,1))

        umpc = minimize(self.ObjectiveFn,x_accelvector,args = (carnow,deernow,setSpeed),bounds = bounds, method = 'SLSQP')
        # umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001})
        #method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001}
        opt_x_accel = umpc.x[0]

        return opt_x_accel

def demo():

    x_acceldistance = 50.0
    setSpeed = 25.0

    deer_ind = '1010000001110010110011110'

    deer_ind = BinaryConversion(deer_ind)

    deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

    # Indicate deer initial position
    deer.x_Deer = 80
    deer.y_Deer = 0#PUT THE DEER IN THE MIDDLE OF THE ROAD!!
        
    # Define simulation time and dt
    simtime = 10
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = 25.0,tiretype='pacejka')


     #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = 0.0 #let the vehicle start away from lane.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    MPC = MPC_G(q_obstacle_error = 1.0,q_x_accel=1.0,q_cruise_speed=1.0)


    steervec = zeros(len(t))
    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]
            opt_x_accel = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

            if (opt_x_accel > 0): 
                gas = opt_x_accel*250
                brake = 0

            else:
                gas = 0
                brake = opt_x_accel*500

            if ((deer.x_Deer - car.x[2]) < x_acceldistance):
                carx[k,:],carxdot[k,:] = car.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                steervec[k] = opt_steer

            else:
                carx[k,:],carxdot[k,:] = car.heuns_update(steer = 0, setspeed = setSpeed)
                deerx[k,:] = deer.updateDeer(car.x[2]) #array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                steervec[k] = opt_steer

            print t[k],opt_steer,deer.y_Deer

    ayg = (carxdot[:,1]+carx[:,5]*carx[:,3])/9.81
    figure()
    plot(t,steervec,'k')
    xlabel('Time (s)')
    ylabel('steer angle (rad)')
    figure()
    plot(t,carx[:,0],'k')
    xlabel('Time (s)')
    ylabel('car Y position (m)')
    figure()
    plot(carx[:,2],carx[:,0],'k',deerx[:,2],deerx[:,3],'ro')
    axis('equal')
    xlabel('X (m)')
    ylabel('Y (m)')
    legend(['car','deer'])
    figure()
    plot(t,ayg,'k')
    xlabel('time (s)')
    ylabel('lateral acceleration (g)')
    show()
if __name__ == '__main__':
    demo()