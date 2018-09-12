from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
from MPC_F import *
from MPC_G import *
import copy

class MPC_H:

    def __init__(self,q_lane_error = 10.0,q_obstacle_error_F = .05,q_lateral_velocity = 0.00,q_steering_effort = 0.0,q_lat_accel = 0.05,q_obstacle_error_G = 1000000000.0, q_x_accel = 0.0, q_cruise_speed = 0.01, gas_max = 1.0, brake_max = 0.5,predictionmethod = 'static'):

        self.q_lane_error = q_lane_error
        self.q_obstacle_error_F = q_obstacle_error_F
        self.q_lateral_velocity = q_lateral_velocity
        self.q_steering_effort = q_steering_effort
        self.q_lat_accel = q_lat_accel
        self.q_obstacle_error_F = q_obstacle_error_F
        self.q_lateral_velocity = q_lateral_velocity
        self.q_steering_effort = q_steering_effort
        self.q_lat_accel = q_lat_accel
        self.q_obstacle_error_G = q_obstacle_error_G
        self.q_x_accel = q_x_accel
        self.q_cruise_speed = q_cruise_speed
        self.gas_max = gas_max
        self.brake_max = brake_max
        self.predictionmethod = predictionmethod

        self.MPCF = MPC_F(q_lane_error = self.q_lane_error,q_obstacle_error = self.q_obstacle_error_F,q_lateral_velocity = self.q_lateral_velocity,q_steering_effort = self.q_obstacle_error_F,q_accel = self.q_lat_accel,predictionmethod = self.predictionmethod)
        self.MPCG = MPC_G(q_obstacle_error = self.q_obstacle_error_G, q_x_accel = self.q_x_accel,q_cruise_speed = self.q_cruise_speed, gas_max = self.gas_max, brake_max = self.brake_max, predictionmethod = self.predictionmethod)


    def calcOptimal(self,carnow,deernow,setSpeed = 25.0,yroad = 0.0):    
        opt_steer = self.MPCF.calcOptimal(carnow, deernow, yroad)
        opt_gas,opt_brake = self.MPCG.calcOptimal(carnow, deernow, setSpeed)
        return opt_gas,opt_brake,opt_steer

def demo_GAdeer():

    x_acceldistance = 50.0
    setSpeed = 25.0

    deer_ind = '1010000001110010110011110'
    deer_ind = '1001000100000000000010000'#trained against E, 
    deer_ind = '000110000000001000100001'#trained against H for 8 generations.


    deer_ind = BinaryConversion(deer_ind)
    print(deer_ind)

    deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])
    #deer.Psi1_Deer = -60*3.14/180.0
    # Indicate deer initial position
    deer.x_Deer = 80
    deer.y_Deer = -2#PUT THE DEER IN THE MIDDLE OF THE ROAD!!
        
    # Define simulation time and dt
    simtime = 10
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = 25.0,tiretype='pacejka')
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = 0.0 #let the vehicle start away from lane.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    MPC = MPC_H(q_lane_error = 10.0,q_obstacle_error_F = .05,q_lateral_velocity = 0.00,q_steering_effort = 0.0,q_lat_accel = 0.005,q_obstacle_error_G = 1000000000.0, q_x_accel = 0.0, q_cruise_speed = 25.0, gas_max = 0.25, brake_max = 0.25, predictionmethod = 'CV')

    steervec = zeros(len(t))
    accelvec = zeros(len(t))
    distancevec = zeros(len(t))
    cafvec = zeros(len(t))
    carvec = zeros(len(t))

    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]
            #accelvec = zeros(5)
            opt_steer = 0

            #MPC.ObjectiveFn(accelvec,car,deer,25.0)
            #opt_x_accel = 0#
            gas,brake,steer = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

            if ((deer.x_Deer - car.x[2]) < x_acceldistance):

                carx[k,:],carxdot[k,:],steervec[k] = car.heuns_update(gas = gas, brake = brake, steer = steer, cruise = 'off')
                deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]
                cafvec[k] = car.Caf
                carvec[k] = car.Car
                print "mpc active"

            else:
                carx[k,:],carxdot[k,:] = car.heuns_update(steer = steer, setspeed = setSpeed,)
                #carx[k,:],carxdot[k,:] = car.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]
                cafvec[k] = car.Caf
                carvec[k] = car.Car
                steervec[k] = 0

            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)
            print round(t[k],2),round(gas,2),round(brake,2)

    ayg = (carxdot[:,1]+carx[:,5]*carx[:,3])/9.81
    axg = (carxdot[:,3]-carx[:,5]*carx[:,1])/9.81
    print(deer_ind)
    

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
    plot(t,cafvec,t,carvec)
    xlabel('time (s)')
    ylabel('Cornering Stiffness (N/rad)')
    legend(['front','rear'])

    figure()
    plot(t,deerx[:,0],'k')
    xlabel('time (s)')
    ylabel('deer speed (m/s)')

    figure()
    plot(t,deerx[:,1],'k')
    xlabel('time (s)')
    ylabel('deer speed (m/s)')

    figure()
    plot(axg,ayg,'k')
    axis('equal')
    xlabel('Forward Acceleration (g)')
    ylabel('Lateral Acceleration (g)')


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
        car_plot.set_xy([car_x[i]-(car_length/2), car_y[i]-(car_width/2)])
        car_plot.angle = car_yaw[i]*180/3.14

        deer_plot.set_width(deer_length)
        deer_plot.set_height(deer_width)
        deer_plot.set_xy([deer_x[i]-(deer_length/2), deer_y[i]-(deer_width/2)])
        deer_plot.angle = 90-deer_yaw[i]*180/3.14

        return car_plot,deer_plot,

    # Run anumation
    anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x),interval=50,blit=True)

    ### ANIMATION END

    show()

def demo_CVdeer():

    x_acceldistance = 50.0
    setSpeed = 25.0

    angle = -78
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


    car = BicycleModel(dT = dt, U = 25.0,tiretype='pacejka')
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = 0.0 #let the vehicle start away from lane.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    MPC = MPC_H(q_lane_error = 10.0,q_obstacle_error_F = .05,q_lateral_velocity = 0.00,q_steering_effort = 0.0,q_lat_accel = 0.005,q_obstacle_error_G = 1000000000.0, q_x_accel = 0.0, q_cruise_speed = 25.0, gas_max = 0.25, brake_max = 0.25, predictionmethod = 'CV')

    steervec = zeros(len(t))
    accelvec = zeros(len(t))
    distancevec = zeros(len(t))
    cafvec = zeros(len(t))
    carvec = zeros(len(t))

    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]
            #accelvec = zeros(5)
            opt_steer = 0

            #MPC.ObjectiveFn(accelvec,car,deer,25.0)
            #opt_x_accel = 0#
            gas,brake,steer = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

            if ((deer.x_Deer - car.x[2]) < x_acceldistance):

                carx[k,:],carxdot[k,:],steervec[k] = car.heuns_update(gas = gas, brake = brake, steer = steer, cruise = 'off')
                deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]
                cafvec[k] = car.Caf
                carvec[k] = car.Car
                steervec[k] = steer
                print "mpc active"

            else:
                carx[k,:],carxdot[k,:],steervec[k] = car.heuns_update(steer = steer, setspeed = setSpeed,)
                #carx[k,:],carxdot[k,:] = car.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]
                cafvec[k] = car.Caf
                carvec[k] = car.Car

            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)
            print round(t[k],2),round(gas,2),round(brake,2)

    ayg = (carxdot[:,1]+carx[:,5]*carx[:,3])/9.81
    axg = (carxdot[:,3]-carx[:,5]*carx[:,1])/9.81
    

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
    plot(t,cafvec,t,carvec)
    xlabel('time (s)')
    ylabel('Cornering Stiffness (N/rad)')
    legend(['front','rear'])

    figure()
    plot(t,deerx[:,0],'k')
    xlabel('time (s)')
    ylabel('deer speed (m/s)')

    figure()
    plot(t,deerx[:,1],'k')
    xlabel('time (s)')
    ylabel('deer speed (m/s)')

    figure()
    plot(axg,ayg,'k')
    axis('equal')
    xlabel('Forward Acceleration (g)')
    ylabel('Lateral Acceleration (g)')


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