from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
from CV_Deer import *
import copy

class MPC_F:
    def __init__(self, Np=5, dtp=.2,q_lane_error = 1.0,q_obstacle_error = 1.0,q_steering_effort=1.0,q_accel = 1.0,q_lateral_velocity=1.0,steering_angle_max=.20, epsilon = 0.00001,downsample_horizon = 'false',predictionmethod = 'static'):
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

    def predictCar(self,carnow,steervector):
        predictCar = copy.deepcopy(carnow)
        predictCar.tiretype='linear'
        if(self.downsample_horizon=='false'):
            xcar_pred_downsampled = zeros((self.Np,6))
            xdotcar_pred_downsampled  = zeros((self.Np,6))
            predictCar.dT = self.dtp
            for k in range(0,self.Np):
                xcar_pred_downsampled[k,:],xdotcar_pred_downsampled[k,:] = predictCar.heuns_update(steer = steervector[k], setspeed = 25.0)
        else:
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
            for k in range(0,len(tvec)):
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
        if (self.predictionmethod == 'static'):
            xdeer_pred = self.predictDeer_static(deernow,carnow)
        else:
            xdeer_pred = self.predictDeer_CV(deernow,carnow)
        #calculate lateral acceleration Vdot+U*psidot for the prediction too, so we can use it in objective function
        car_y_accel_pred = xdotcar_pred[:,1]+xcar_pred[:,3]*xcar_pred[:,5]
        #Np rows by 5 columns, one for x and y of deer
        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.
        for k in range(0,self.Np):
            distance = 1.0/(sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2)+self.epsilon)#+ (xcar_pred[k,2] - xdeer_pred[k,2])**2+self.epsilon)
            #return distance
            if(carnow.x[2]<deernow.x_Deer):
                J = J +  self.q_lateral_velocity*(xcar_pred[k,1])**2+self.q_steering_effort * (steervector[k])**2 + self.q_lane_error * (xcar_pred[k,0]-yroad)**2 + self.q_obstacle_error * (distance)**2 + self.q_accel*((car_y_accel_pred[k]))**2
            else:
                #print "passed deer!"
                J = J +  self.q_steering_effort * (steervector[k])**2 + self.q_lane_error * (xcar_pred[k,0]-yroad)**2+ self.q_accel*((car_y_accel_pred[k]))**2
        return J

    def calcDist(self,steervector,carnow,deernow):
        xcar_pred,junk = self.predictCar(carnow,steervector)
        if (self.predictionmethod == 'static'):
            xdeer_pred = self.predictDeer_static(deernow,carnow)
        else:
            xdeer_pred = self.predictDeer_CV(deernow,carnow)
        #Np rows by 5 columns, one for x and y of deer
        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.
        distance=zeros(self.Np)
        for k in range(0,self.Np):
            #print k
            distance[k] = sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2 + (xcar_pred[k,2] - xdeer_pred[k,2])**2)
            #print distance[k]

        #print min(distance)
        return min(distance)-3

    def calcOptimal(self,carnow, deernow,yroad):
        steervector = 0.1*random.randn(self.Np)

        bounds = [(-self.steering_angle_max,self.steering_angle_max)]
        for ind in range(1,self.Np):
            bounds.insert(0,(-self.steering_angle_max,self.steering_angle_max))

        cons = ({'type': 'ineq','fun':self.calcDist, 'args':(carnow,deernow)})

        umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method = 'SLSQP',constraints=cons)
        # umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001})
        #method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001}
        opt_steering = umpc.x[0]

        return opt_steering

def demo_GAdeer():

    swerveDistance = 50.0
    setSpeed = 25.0

    deer_ind = '110111111001110010001001'

    deer_ind = BinaryConversion(deer_ind)

    deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

    # Indicate deer initial position
    deer.x_Deer = 80
    deer.y_Deer = -2.0#PUTrsion(deer_ind)
        
    # Define simulation time and dt
    simtime = 5
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = 25.0,tiretype='linear')


     #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = -0.0 #let the vehicle start away from lane.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    #MPC = MPC_F(q_lane_error = 10.0,q_obstacle_error = 5000000.0,q_lateral_velocity=0.00,q_steering_effort=0.0,q_accel = 0.005)
    weight = 10.0
    MPC = MPC_F(q_lane_error = weight,q_obstacle_error =1.0/weight*10,q_lateral_velocity=0.00,q_steering_effort=0.0,q_accel = 0.005,predictionmethod='CV')

    steervec = zeros(len(t))
    cafvec = zeros(len(t))
    carvec = zeros(len(t))
    distancevec = zeros(len(t))

    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]

            if ((deer.x_Deer - car.x[2]) < swerveDistance): 
                ##### The commented lines below allow you to test the objective function independently
                # steervector = 0.01*random.randn(MPC.Np)
                # bounds = [(-MPC.steering_angle_max,MPC.steering_angle_max)]
                # for ind in range(1,MPC.Np):
                #     bounds.insert(0,(-MPC.steering_angle_max,MPC.steering_angle_max))
                # opt_steer = 0
                # #steervector,carnow,deernow,yroad
                # J = MPC.ObjectiveFn(steervector,car,deer,yroad=0)
                opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
            else:
                opt_steer = 0

            carx[k,:],carxdot[k,:] = car.heuns_update(steer = opt_steer, setspeed = 25.0)
            cafvec[k] = car.Caf
            carvec[k] = car.Car
            #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
            deerx[k,:] = deer.updateDeer(car.x[2])
            steervec[k] = opt_steer
            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

            print round(t[k],2),round(opt_steer,2),round(deer.y_Deer,2)


    ## SAVE xcar and xdeer

    TestNumber = 1
    FileName ='Test/Test' + str(TestNumber) + '.csv';

    newFile = open(FileName,'w+');
    newFile.close();
    newFile = open(FileName, 'a');

    for ind2 in range(0,6):
        for ind1 in range(0, len(carx)):
            newFile.write(str(carx[ind1,ind2]) + ' ');
        newFile.write('\n')
    for ind2 in range(0,4):
        for ind1 in range(0, len(deerx)):
            newFile.write(str(deerx[ind1,ind2]) + ' ');
        newFile.write('\n')

    ## SAVE end


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
    xlabel('X (m)')
    ylabel('Y (m)')
    ylim([-5,5])
    legend(['car','deer'])
    figure()
    plot(t,ayg,'k')
    xlabel('time (s)')
    ylabel('lateral acceleration (g)')
    figure()
    plot(t,cafvec,t,carvec)
    xlabel('time (s)')
    ylabel('Cornering Stiffness (N/rad)')
    legend(['front','rear'])
    figure()
    plot(t,distancevec)
    xlabel('time (s)')
    ylabel('Distance(m)')

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
    ax.set_xlim(0, 150)
    ax.set_ylim(-25, 25)

    # Initialize rectangles
    car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='b', alpha = 0.5)
    deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='r', alpha = 0.5)
    background = patches.Rectangle((-100,-100),200,200,fc='k')
    center_line_1 = patches.Rectangle((-10,(1.75+0.025)),200,0.1,fc='y')
    center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),200,0.1,fc='y')
    right_line = patches.Rectangle((-10,-1.75),200,0.1,fc='w')
    left_line = patches.Rectangle((-10,4.75),200,0.1,fc='w')

    def init():
        ax.add_patch(car_plot)
        ax.add_patch(deer_plot)
        ax.add_patch(background)
        ax.add_patch(left_line)
        ax.add_patch(right_line)
        ax.add_patch(center_line_1)
        ax.add_patch(center_line_2)
        return car_plot,deer_plot,

    # Set animation
    def animate(i):
        car_plot.set_width(car_length)
        car_plot.set_height(car_width)
        car_plot.set_xy([car_x[i]-(car_length/2), car_y[i]-(car_width/2)])
        car_plot.angle = car_yaw[i]*180/3.14

        deer_plot.set_width(deer_length)
        deer_plot.set_height(deer_width)
        deer_plot.set_xy([deer_x[i]-(deer_length/2*sin(deer_yaw[i])), deer_y[i]]-(deer_width/2*cos(deer_yaw[i])))
        deer_plot.angle = 90-deer_yaw[i]*180/3.14

        return car_plot,deer_plot,

    # Run anumation
    anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x),interval=20,blit=True)

    ### ANIMATION END

    show()

def demo_CVdeer():

    swerveDistance = 50.0
    setSpeed = 25.0

    angle = -77
    speed = 5
    # Initiate process
    deer = CV_Deer()
    deer.x_Deer = 80
    deer.y_Deer = -2
    deer.Psi_Deer = angle*3.1415/180
    deer.Speed_Deer = speed
        
    # Define simulation time and dt
    simtime = 10
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = 25.0,tiretype='linear')


     #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = -0.0 #let the vehicle start away from lane.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    #MPC = MPC_F(q_lane_error = 10.0,q_obstacle_error = 5000000.0,q_lateral_velocity=0.00,q_steering_effort=0.0,q_accel = 0.005)
    weight = 10.0
    MPC = MPC_F(q_lane_error = weight,q_obstacle_error =1.0/weight*10,q_lateral_velocity=0.00,q_steering_effort=0.0,q_accel = 0.005,predictionmethod='CV')

    steervec = zeros(len(t))
    cafvec = zeros(len(t))
    carvec = zeros(len(t))
    distancevec = zeros(len(t))

    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]

            if ((deer.x_Deer - car.x[2]) < swerveDistance): 
                ##### The commented lines below allow you to test the objective function independently
                # steervector = 0.01*random.randn(MPC.Np)
                # bounds = [(-MPC.steering_angle_max,MPC.steering_angle_max)]
                # for ind in range(1,MPC.Np):
                #     bounds.insert(0,(-MPC.steering_angle_max,MPC.steering_angle_max))
                # opt_steer = 0
                # #steervector,carnow,deernow,yroad
                # J = MPC.ObjectiveFn(steervector,car,deer,yroad=0)
                opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
            else:
                opt_steer = 0

            carx[k,:],carxdot[k,:] = car.heuns_update(steer = opt_steer, setspeed = 25.0)
            cafvec[k] = car.Caf
            carvec[k] = car.Car
            #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
            deerx[k,:] = deer.updateDeer(car.x[2])
            steervec[k] = opt_steer
            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

            print round(t[k],2),round(opt_steer,2),round(deer.y_Deer,2)


    ## SAVE xcar and xdeer

    TestNumber = 1
    FileName ='Test/Test' + str(TestNumber) + '.csv';

    newFile = open(FileName,'w+');
    newFile.close();
    newFile = open(FileName, 'a');

    for ind2 in range(0,6):
        for ind1 in range(0, len(carx)):
            newFile.write(str(carx[ind1,ind2]) + ' ');
        newFile.write('\n')
    for ind2 in range(0,4):
        for ind1 in range(0, len(deerx)):
            newFile.write(str(deerx[ind1,ind2]) + ' ');
        newFile.write('\n')

    ## SAVE end


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
    xlabel('X (m)')
    ylabel('Y (m)')
    ylim([-5,5])
    legend(['car','deer'])
    figure()
    plot(t,ayg,'k')
    xlabel('time (s)')
    ylabel('lateral acceleration (g)')
    figure()
    plot(t,cafvec,t,carvec)
    xlabel('time (s)')
    ylabel('Cornering Stiffness (N/rad)')
    legend(['front','rear'])
    figure()
    plot(t,distancevec)
    xlabel('time (s)')
    ylabel('Distance(m)')

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
    car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='b', alpha = 0.5)
    deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='r', alpha = 0.5)
    background = patches.Rectangle((-100,-100),200,200,fc='k')
    center_line_1 = patches.Rectangle((-10,(1.75+0.025)),200,0.1,fc='y')
    center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),200,0.1,fc='y')
    right_line = patches.Rectangle((-10,-1.75),200,0.1,fc='w')
    left_line = patches.Rectangle((-10,4.75),200,0.1,fc='w')

    def init():
        ax.add_patch(car_plot)
        ax.add_patch(deer_plot)
        ax.add_patch(background)
        ax.add_patch(left_line)
        ax.add_patch(right_line)
        ax.add_patch(center_line_1)
        ax.add_patch(center_line_2)
        return car_plot,deer_plot,

    # Set animation
    def animate(i):
        car_plot.set_width(car_length)
        car_plot.set_height(car_width)
        car_plot.set_xy([car_x[i]-(car_length/2), car_y[i]-(car_width/2)])
        car_plot.angle = car_yaw[i]*180/3.14

        deer_plot.set_width(deer_length)
        deer_plot.set_height(deer_width)
        deer_plot.set_xy([deer_x[i]-(deer_length/2*sin(deer_yaw[i])), deer_y[i]]-(deer_width/2*cos(deer_yaw[i])))
        deer_plot.angle = 90-deer_yaw[i]*180/3.14

        return car_plot,deer_plot,

    # Run anumation
    anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x),interval=20,blit=True)

    ### ANIMATION END

    show()



if __name__ == '__main__':
    demo_CVdeer()







        
