from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
import copy

class MPC_F:
    def __init__(self, Np=20, dtp=.2,q_lane_error = 1.0,q_obstacle_error = 1.0,q_steering_effort=1.0,q_accel = 1.0,q_lateral_velocity=1.0,steering_angle_max=.20, epsilon = 0.00001):
        self.Np = Np
        self.dtp = dtp
        self.q_lane_error = q_lane_error
        self.q_obstacle_error = q_obstacle_error
        self.q_steering_effort = q_steering_effort
        self.steering_angle_max = steering_angle_max
        self.epsilon = epsilon
        self.prediction_time = self.dtp*self.Np
        self.t_horizon = arange(0,self.prediction_time,self.dtp)#go to one extra so the horizon matches
        self.q_accel = q_accel
        self.q_lateral_velocity = q_lateral_velocity

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

    def predictCar(self,carnow,steervector):
        predictCar = copy.deepcopy(carnow)
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
        steervector_upsampled = interp(tvec,self.t_horizon,steervector)
        #print steervector_upsampled.shape
        #actually predict the car's states given the input
        for k in range(0,self.Np):
            xcar_pred[k,:],xdotcar_pred[k,:] = predictCar.heuns_update(steer = steervector_upsampled[k], setspeed = 25.0)
        #now downsample the prediction so it is only MPC.Np points long
        for k in range(0,6):
            xcar_pred_downsampled[:,k] = interp(self.t_horizon,tvec,xcar_pred[:,k])
            xdotcar_pred_downsampled[:,k] = interp(self.t_horizon,tvec,xdotcar_pred[:,k])
        #print xdotcar_pred_downsampled.shape,xcar_pred_downsampled.shape
        return xcar_pred_downsampled,xdotcar_pred_downsampled

    def ObjectiveFn(self,steervector,carnow,deernow,yroad):
        J=0
        #Np rows by 6 columns, one for each state (or vice versa)
        xcar_pred,xdotcar_pred = self.predictCar(carnow,steervector)
        xdeer_pred = self.predictDeer_static(deernow,carnow)
        #calculate lateral acceleration Vdot+U*psidot for the prediction too, so we can use it in objective function
        car_y_accel_pred = xdotcar_pred[:,1]+xcar_pred[:,3]*xcar_pred[:,5]
        #Np rows by 5 columns, one for x and y of deer
        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.
        for k in range(0,self.Np):
            distance = sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2+ (xcar_pred[k,2] - xdeer_pred[k,2])**2)
            #return distance
            if(carnow.x[2]<deernow.x_Deer):
                J = J +  self.q_lateral_velocity*(xcar_pred[k,1])**2+self.q_steering_effort * (steervector[k])**2 + self.q_lane_error * (xcar_pred[k,0]-yroad)**2 + self.q_obstacle_error * ((distance+self.epsilon))**2 + self.q_accel*((car_y_accel_pred[k]))**2
            else:
                #print "passed deer!"
                J = J +  self.q_steering_effort * (steervector[k])**2 + self.q_lane_error * (xcar_pred[k,0]-yroad)**2+ self.q_accel*((car_y_accel_pred[k]))**2
        return J


    def calcOptimal(self,carnow, deernow,yroad):
        steervector = 0.1*random.randn(self.Np)

        bounds = [(-self.steering_angle_max,self.steering_angle_max)]
        for ind in range(1,self.Np):
            bounds.insert(0,(-self.steering_angle_max,self.steering_angle_max))

        umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method = 'SLSQP')
        # umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001})
        #method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001}
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
    deer.y_Deer = 0#PUT THE DEER IN THE MIDDLE OF THE ROAD!!
        
    # Define simulation time and dt
    simtime = 10
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = 25.0,tiretype='linear')


     #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = 1.5 #let the vehicle start away from lane.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    MPC = MPC_F(q_lane_error = 50.0,q_obstacle_error = 1.0,q_lateral_velocity=0.00,q_steering_effort=0.0,q_accel = 0.05)


    steervec = zeros(len(t))
    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]

            if ((deer.x_Deer - car.x[2]) < swerveDistance): 
            #     ##### The commented lines below allow you to test the objective function independently
            #     steervector = 0.01*random.randn(MPC.Np)
            #     bounds = [(-MPC.steering_angle_max,MPC.steering_angle_max)]
            #     for ind in range(1,MPC.Np):
            #         bounds.insert(0,(-MPC.steering_angle_max,MPC.steering_angle_max))
            #     opt_steer = 0
            #     #steervector,carnow,deernow,yroad
            #     J = MPC.ObjectiveFn(steervector,car,deer,yroad=0)
            #     print J
                opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
            else:
                opt_steer = 0

            carx[k,:],carxdot[k,:] = car.heuns_update(steer = opt_steer, setspeed = 25.0)
            deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
            steervec[k] = opt_steer

            print t[k],opt_steer,deer.y_Deer
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
    show()
if __name__ == '__main__':
    demo()







        
