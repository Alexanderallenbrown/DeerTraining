from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
import copy

class MPC_G:
    def __init__(self, Np=5, dtp=.2, q_obstacle_error = 1.0, q_cruise_speed = 1.0, q_x_accel=1.0, gas_max=1.0, brake_max = .5, epsilon = 0.0001,downsample_horizon = 'false',predictionmethod = 'static'):
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
        self.downsample_horizon = downsample_horizon
        self.predictionmethod = predictionmethod

    def predictDeer_static(self,deernow,carnow):
        predictDeer = copy.deepcopy(deernow)
        if(self.downsample_horizon=='false'):
            predictDeer.dT = self.dtp
            xdeer_pred_downsampled = zeros((self.Np,4))
            for k in range(0,self.Np):
                #eventually, the deer will need to also have a model of how the CAR moves...
                xdeer_pred_downsampled[k,:] = predictDeer.xdeer#array([predictDeer.Speed_Deer,predictDeer.Psi_Deer,predictDeer.x_Deer,predictDeer.y_Deer])
        else:
            
            #make a time vector for prediction using 'fine' timestep of deer
            tvec = arange(0,self.prediction_time+predictDeer.dT,predictDeer.dT)
            #initialize the 'fine' predicted state vector
            xdeer_pred = zeros((len(tvec),4))
            for k in range(0,len(tvec)):
                #eventually, the deer will need to also have a model of how the CAR moves...
                xdeer_pred[k,:] = predictDeer.xdeer#array([predictDeer.Speed_Deer,predictDeer.Psi_Deer,predictDeer.x_Deer,predictDeer.y_Deer])
            #now downsample the prediction so that the horizon matches MPC rather than the deer
            #this way, the MPC will only look at and attempt to optimize a few points, but the prediction will be high-fi
            xdeer_pred_downsampled = zeros((self.Np,4))
            for k in range(0,4):
                xdeer_pred_downsampled[:,k] = interp(self.t_horizon,tvec,xdeer_pred[:,k])
        return xdeer_pred_downsampled

    # def predictDeer_static(self,deernow,carnow):
    #     predictDeer = copy.deepcopy(deernow)
    #     #make a time vector for prediction using 'fine' timestep of deer
    #     tvec = arange(0,self.prediction_time+predictDeer.dT,predictDeer.dT)
    #     #initialize the 'fine' predicted state vector
    #     xdeer_pred = zeros((len(tvec),4))
    #     for k in range(0,len(tvec)):
    #         #eventually, the deer will need to also have a model of how the CAR moves...
    #         xdeer_pred[k,:] = predictDeer.xdeer
    #     #now downsample the prediction so that the horizon matches MPC rather than the deer
    #     #this way, the MPC will only look at and attempt to optimize a few points, but the prediction will be high-fi
    #     xdeer_pred_downsampled = zeros((self.Np,4))
    #     for k in range(0,4):
    #         xdeer_pred_downsampled[:,k] = interp(self.t_horizon,tvec,xdeer_pred[:,k])
    #     return xdeer_pred_downsampled

    def predictDeer_CV(self,deernow,carnow):
        predictDeer = copy.deepcopy(deernow)
        if(self.downsample_horizon=='false'):
            predictDeer.dT = self.dtp
            xdeer_pred_downsampled = zeros((self.Np,4))
            #note: the deer's global coordinate system is rotated by 90 degrees compared to car.... :(  >:(
            xvel = predictDeer.xdeer[0]*sin(predictDeer.xdeer[1])
            yvel = predictDeer.xdeer[0]*cos(predictDeer.xdeer[1])
            for k in range(0,self.Np):
                #eventually, the deer will need to also have a model of how the CAR moves...
                xdeer_pred_downsampled[k,:] = predictDeer.xdeer#array([predictDeer.Speed_Deer,predictDeer.Psi_Deer,predictDeer.x_Deer,predictDeer.y_Deer])
                xdeer_pred_downsampled[k,2] += xvel*k*self.dtp
                xdeer_pred_downsampled[k,3] += yvel*k*self.dtp
        else:
            
            #make a time vector for prediction using 'fine' timestep of deer
            tvec = arange(0,self.prediction_time+predictDeer.dT,predictDeer.dT)
            #initialize the 'fine' predicted state vector
            xdeer_pred = zeros((len(tvec),4))
            xvel = predictDeer.xdeer[0]*sin(predictDeer.xdeer[1])
            yvel = predictDeer.xdeer[0]*cos(predictDeer.xdeer[1])
            for k in range(0,len(tvec)):
                #eventually, the deer will need to also have a model of how the CAR moves...
                xdeer_pred[k,:] = predictDeer.xdeer#array([predictDeer.Speed_Deer,predictDeer.Psi_Deer,predictDeer.x_Deer,predictDeer.y_Deer])
                xdeer_pred_downsampled[k,2] += xvel*k*predictDeer.dT
                xdeer_pred_downsampled[k,3] += yvel*k*predictDeer.dT
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
        x_accelvector_upsampled = interp(tvec,self.t_horizon,x_accelvector)
        #print steervector_upsampled.shape
        #actually predict the car's states given the input
        for k in range(0,self.Np):
            if x_accelvector_upsampled[k] > 0:
                gas = x_accelvector_upsampled[k]
                brake = 0
                xcar_pred[k,:],xdotcar_pred[k,:] = predictCar.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
            else:
                gas = 0
                brake = abs(x_accelvector_upsampled[k])
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
        if (self.predictionmethod == 'static'):
            xdeer_pred = self.predictDeer_static(deernow,carnow)
        else:
            xdeer_pred = self.predictDeer_CV(deernow,carnow)

        #Np rows by 5 columns, one for x and y of deer
        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.
        for k in range(0,self.Np):
            distance = sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2+ (xcar_pred[k,2] - xdeer_pred[k,2])**2)
            #return distance
            if(carnow.x[2]<deernow.x_Deer):
                J = J + self.q_x_accel * (x_accelvector[k])**2 + self.q_cruise_speed * (xcar_pred[k,3]-setSpeed)**2 + self.q_obstacle_error * (1/(distance+self.epsilon))**2
            else:
                #print "passed deer!"
                J = J + self.q_cruise_speed * (xcar_pred[k,3]-setSpeed)**2
        return J


    def calcOptimal(self,carnow,deernow,setSpeed):
        x_accelvector = 0.005*random.randn(self.Np)

        bounds = [(-self.brake_max,self.gas_max)]
        for ind in range(1,self.Np):
            bounds.insert(0,(-self.brake_max,self.gas_max))

        umpc = minimize(self.ObjectiveFn,x_accelvector,args = (carnow,deernow,setSpeed),bounds = bounds, method = 'SLSQP')
        # umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001})
        #method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001}
        opt_x_accel = umpc.x[0]

        if (opt_x_accel > 0): 
            gas = opt_x_accel
            brake = 0

        else:
            gas = 0
            brake = abs(opt_x_accel)

        return gas,brake

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

    MPC = MPC_G(q_obstacle_error = 1000000000.0,q_x_accel=0.0,q_cruise_speed=0.01,brake_max = 0.5,predictionmethod='CV')


    steervec = zeros(len(t))
    accelvec = zeros(len(t))
    distancevec = zeros(len(t))
    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]
            #accelvec = zeros(5)
            opt_steer = 0

            #MPC.ObjectiveFn(accelvec,car,deer,25.0)
            #opt_x_accel = 0#
            gas,brake = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

            if ((sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2) < x_acceldistance) and (deer.x_Deer>car.x[2])):

                carx[k,:],carxdot[k,:] = car.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]
                print "mpc active"

            else:
                carx[k,:],carxdot[k,:] = car.heuns_update(steer = 0, setspeed = setSpeed,)
                #carx[k,:],carxdot[k,:] = car.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]

            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)
            print round(t[k],2),round(gas,2),round(brake,2)

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
    plot(t,ayg,'k')
    xlabel('time (s)')
    ylabel('lateral acceleration (g)')
    figure()
    plot(t,carx[:,3],'k')
    xlabel('Time (s)')
    ylabel('car forward velocity (m/s)')
    figure()
    plot(carx[:,2],carx[:,0],'k',deerx[:,2],deerx[:,3],'ro')
    axis('equal')
    xlabel('X (m)')
    ylabel('Y (m)')
    legend(['car','deer'])
    figure()
    plot(t,accelvec,'k')
    xlabel('time (s)')
    ylabel('longitudinal acceleration (m/s/s)')
    figure()
    plot(t,distancevec,'k')
    xlabel('time (s)')
    ylabel('car-deer distance (m)')

    ### CREATE ANIMATION
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib import animation

    # Define parameters
    deer_length = 1.5 # meters
    deer_width = 0.5 # meters
    car_length = 4.5 # meters
    car_width = 2.0 # meters

    # Create car vectors to be used
    car_x = carx[:,2]
    car_y = carx[:,0]
    car_yaw = carx[:,4]

    # Create deer vectors to be used
    deer_x = deerx[:,2]
    deer_y = deerx[:,3]
    deer_yaw = deerx[:,1]

    # Create figure
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.axis('equal')
    ax.set_xlim(0, 100)
    ax.set_ylim(-25, 25)

    # Initialize rectangles
    car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='k')
    deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='y')

    def init():
        ax.add_patch(car_plot)
        ax.add_patch(deer_plot)
        return car_plot,deer_plot,

    # Set animation
    def animate(i):
        car_plot.set_width(car_length)
        car_plot.set_height(car_width)
        car_plot.set_xy([car_x[i]-(car_length/2*cos(car_yaw[i])), car_y[i]-(car_width/2*sin(car_yaw[i]))])
        car_plot.angle = car_yaw[i]*180/3.14

        deer_plot.set_width(deer_length)
        deer_plot.set_height(deer_width)
        deer_plot.set_xy([deer_x[i]-(deer_length/2*sin(deer_yaw[i])), deer_y[i]]-(deer_width/2*cos(deer_yaw[i])))
        deer_plot.angle = 90-deer_yaw[i]*180/3.14

        return car_plot,deer_plot,

    # Run anumation
    anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x),interval=50,blit=True)

    ### ANIMATION END

    show()


if __name__ == '__main__':
    demo()