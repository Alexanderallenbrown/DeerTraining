from numpy import *
from scipy.optimize import minimize
from matplotlib.pyplot import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
from CV_Deer import *
import copy

class PointMassVehicle:
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

        return array([self.y,self.ydot,self.x,self.xdot])

class MPC_PM:
    def __init__(self, Np=10, dtp=.1,q_lane_error = 1.0,q_obstacle_error = 1.0,q_steering_effort=1.0,q_accel = 1.0,q_lateral_velocity=1.0,q_x_accel = 1.0,q_cruise_speed = 1.0,uv_max=0.6*9.81, epsilon = 0.00001,downsample_horizon = 'false',predictionmethod = 'static'):
        self.Np = Np
        self.dtp = dtp
        self.q_lane_error = q_lane_error
        self.q_obstacle_error = q_obstacle_error
        self.q_steering_effort = q_steering_effort
        self.uv_max = uv_max
        self.epsilon = epsilon
        self.prediction_time = self.dtp*self.Np
        self.t_horizon = arange(0,self.prediction_time,self.dtp)#go to one extra so the horizon matches
        self.q_accel = q_accel
        self.q_lateral_velocity = q_lateral_velocity
        self.predictionmethod = predictionmethod
        self.q_x_accel = q_x_accel
        self.q_cruise_speed = q_cruise_speed

    def predictCar(self,carnow,inputvector):
        predictCar = PointMassVehicle(x=carnow.x[2], y = carnow.x[0], xdot = carnow.x[3], ydot = carnow.x[1], dT = carnow.dT )
        
        xcar_pred = zeros((self.Np,4))
        xdotcar_pred = zeros((self.Np,4))

        uvec = inputvector

        for k in range(0,self.Np):
            u = uvec[k]
            xcar_pred[k,:] = predictCar.updateStates(u)
            xdotcar_pred[k,:] = array([xcar_pred[k,1],u,xcar_pred[k,3],0])

        return xcar_pred, xdotcar_pred

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

        return xdeer_pred_downsampled

    constrain_y = 2

    def ObjectiveFn(self,uvec,carnow,deernow,yroad):
        J=0
        #Np rows by 6 columns, one for each state (or vice versa)
        xcar_pred,xdotcar_pred = self.predictCar(carnow,uvec)
        if (self.predictionmethod == 'static'):
            xdeer_pred = self.predictDeer_static(deernow,carnow)
        else:
            xdeer_pred = self.predictDeer_CV(deernow,carnow)
        #calculate lateral acceleration Vdot+U*psidot for the prediction too, so we can use it in objective function
        car_y_accel_pred = xdotcar_pred[:,1] #+xcar_pred[:,3]*xcar_pred[:,5]

        self.car_y_accel_pred = car_y_accel_pred

        #Np rows by 5 columns, one for x and y of deer
        J = 0 # initialize the objective to zero
        #now loop through and upfdate J for every timestep in the prediction horizon.

        #optimizer variables
        K1 = .1*ones(self.Np)#weight on input
        K2 = 10*ones(self.Np)#weight on y vehicle
        K3 = zeros(self.Np);K3[-1] = 1000#weight on terminal y-velocity. Only Penalize TERMINAL!!
        K4 = 100*ones(self.Np)#weight on distance from obstacle.

        for k in range(0,self.Np):
            distance = 1.0/(sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2)+self.epsilon)#+ (xcar_pred[k,2] - xdeer_pred[k,2])**2+self.epsilon)
            #return distance
            if(carnow.x[2]<deernow.x_Deer):
                J = J + K1[k]*uvec[k]**2 + K2[k]*(xcar_pred[k,0]-yroad)**2+ K3[k]*xcar_pred[k,1]**2+K4[k] * distance**2
            else:
                J = J + K1[k]*uvec[k]**2 + K2[k]*(xcar_pred[k,0]-yroad)**2+ K3[k]*xcar_pred[k,1]**2
          
        return J



    def calcDist(self,inputvector,carnow,deernow):
        # Predict the future location of the car
            xcar_pred, xdotcar_pred = self.predictCar(carnow,inputvector)
            if (self.predictionmethod == 'static'):
                xdeer_pred = self.predictDeer_static(deernow,carnow)
            else:
                xdeer_pred = self.predictDeer_CV(deernow,carnow)
            #Np rows by 5 columns, one for x and y of deer
            distance=zeros(self.Np)
            for k in range(0,self.Np):
                # print k
                distance[k] = sqrt((xcar_pred[k,0] - xdeer_pred[k,3])**2 + (xcar_pred[k,2] - xdeer_pred[k,2])**2)
                #print distance[k]

            #print min(distance)
            return min(distance)-2



    def stayInRoadRight(self,inputvector,carnow):
        # Predict the future location of the car
        xcar_pred, xdotcar_pred = self.predictCar(carnow,inputvector)

        # Find the minimum and maximum values for the predicted y-position
        min_y = min(xcar_pred[:,0])

        # Determine the min and max allowable y-positions
        min_allow_y = -1.75-4

        return min_y-min_allow_y

    def stayInRoadLeft(self,inputvector,carnow):
        # Predict the future location of the car
        xcar_pred, xdotcar_pred = self.predictCar(carnow,inputvector)

        # Find the minimum and maximum values for the predicted y-position
        max_y = max(xcar_pred[:,0])

        # Determine the min and max allowable y-positions
        max_allow_y = 1.75+3.5+1.5

        return max_allow_y-max_y

    def calcOptimal(self,carnow, deernow,yroad):

        uvec = 0.1*random.randn(self.Np)

        bounds = [(-self.uv_max,self.uv_max)]
        for ind in range(1,self.Np):
            bounds.insert(0,(-self.uv_max,self.uv_max))

        cons = ({'type': 'ineq','fun':self.calcDist, 'args':(carnow,deernow)},{'type': 'ineq','fun':self.stayInRoadRight, 'args':(carnow,)},{'type': 'ineq','fun':self.stayInRoadLeft, 'args':(carnow,)})

        umpc = minimize(self.ObjectiveFn,uvec,args = (carnow,deernow,yroad),bounds = bounds, method = 'SLSQP',constraints=cons, options ={'maxiter': 100})

        opt_u = umpc.x[0]

        if (isnan(umpc.x[0])==True):
            print "Collision unavoidable: Eliminate collision constraint"
            # Eliminate collision constraint
            cons = ({'type': 'ineq','fun':self.stayInRoadRight, 'args':(carnow,)},{'type': 'ineq','fun':self.stayInRoadLeft, 'args':(carnow,)})
            # Re-do minimization
            umpc = minimize(self.ObjectiveFn,uvec,args = (carnow,deernow,yroad),bounds = bounds, method = 'SLSQP', constraints=cons, options ={'maxiter': 100})
            # Recalculate opt_steering
            opt_u = umpc.x[0]
            
            if (isnan(umpc.x[0])==True):
                print "Impossible to stay in lane: Eliminate lane constraint"
                # Re-do minimization
                umpc = minimize(self.ObjectiveFn,uvec,args = (carnow,deernow,yroad),bounds = bounds, method = 'SLSQP', options ={'maxiter': 100})
                # Recalculate opt_steering
                opt_u = umpc.x[0]
        # umpc = minimize(self.ObjectiveFn,steervector,args = (carnow,deernow,yroad),bounds = bounds, method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001})
        #method='BFGS',options={'xtol': 1e-12, 'disp': False,'eps':.0001,'gtol':.0001}

        opt_steering = opt_u*2.3/(carnow.x[3]**2) #(opt_u + carnow.x[1]*carnow.x[5]*carnow.x[4])*2.3/((carnow.x[3])**2)

        print "opt steering"
        print opt_steering

        return opt_steering


def demo_CVdeer():

    swerveDistance = 80.0
    setSpeed = 25.0

    angle = -87
    speed = 15
    # Initiate process
    deer = CV_Deer()
    deer.x_Deer = 80
    deer.y_Deer = -2
    deer.Psi_Deer = angle*3.1415/180
    deer.Speed_Deer = speed
        
    # Define simulation time and dt
    simtime = 5
    dt = 1/60.0    
    last_command_t = -0.1

    deer.dT = dt
    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = 25.0,tiretype='linear', steering_actuator = 'on')


     #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = 0.0#let the vehicle start away from lane.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    #MPC = MPC_F(q_lane_error = 10.0,q_obstacle_error = 5000000.0,q_lateral_velocity=0.00,q_steering_effort=0.0,q_accel = 0.005)
    weight = 10.0
    MPC = MPC_PM(q_lane_error = weight,q_obstacle_error =1.0,q_lateral_velocity=1.0,q_steering_effort=1.0,q_accel = 1.0,predictionmethod='CV')

    actual_steervec = zeros(len(t))
    command_steervec = zeros(len(t))
    cafvec = zeros(len(t))
    carvec = zeros(len(t))
    distancevec = zeros(len(t))
    opt_accel = 0
    opt_steer = 0
    last_steer_t = 0
    accvec = zeros(len(t))


    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]
            print t[k]

            if ((deer.x_Deer - car.x[2]) < swerveDistance): 
                ##### The commented lines below allow you to test the objective function independently
                # steervector = 0.01*random.randn(MPC.Np)
                # bounds = [(-MPC.steering_angle_max,MPC.steering_angle_max)]
                # for ind in range(1,MPC.Np):
                #     bounds.insert(0,(-MPC.steering_angle_max,MPC.steering_angle_max))
                # opt_steer = 0
                # #steervector,carnow,deernow,yroad
                # J = MPC.ObjectiveFn(steervector,car,deer,yroad=0)
                if ((t[k]- last_steer_t) >= MPC.dtp):

                    opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0.0)
                    if opt_accel >= 0:
                        gas = opt_accel
                        brake = 0
                    else:
                        gas = 0
                        brake = opt_accel
                    print opt_steer, gas, brake
                    last_steer_t = t[k]


            
            #if((deer.x_Deer<car.x[2])):

#                yr = 0
 #               steer_gain = 0.01
  #              L = 20
   #             yc = car.x[0]
    #            fi = car.x[4]
     #           ep = yc + L*sin(fi)
      #          opt_steer = steer_gain*(yr-ep)
        
            else:
                opt_steer = 0
                gas = 0
                brake = 0


            accvec[k]=opt_accel 
            carx[k,:],carxdot[k,:],actual_steervec[k] = car.rk_update(steer = opt_steer, gas = gas, brake = brake, cruise = 'off')
            cafvec[k] = car.Caf
            carvec[k] = car.Car
            #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
            deerx[k,:] = deer.updateDeer(car.x[2])
            command_steervec[k] = opt_steer
            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)



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
    plot(t,actual_steervec,'k',t,command_steervec,'r')
    xlabel('Time (s)')
    ylabel('steer angle (rad)')
    figure()
    plot(t,accvec)
    xlabel('Time (s)')
    ylabel('Acceleration (m/s^2)')

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

def demo_CVdeer_PM():

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

    opt_u = umpc.x

    vehiclesimulate = PointMassVehicle(xv0,yv0,xdotv,ydotv0,dtP,uvmax)
    xvehicle_store = array([xv0,xdotv,yv0,ydotv0])
    t = linspace(0,(Np-1)*dtP,Np)
    for ind in range(1,len(t)):
        xvehicle = vehiclesimulate.updateStates(opt_u)
        xvehicle_store = vstack((xvehicle_store,xvehicle))

if __name__ == '__main__':
    demo_CVdeer()







        