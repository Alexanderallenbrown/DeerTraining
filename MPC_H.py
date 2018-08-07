from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
from BinaryConversion import *
from MPC_F import *
from MPC_G import *
import copy

class MPC_H:

    def __init__(self,q_lane_error = 10.0,q_obstacle_error_F = 5000000.0,q_lateral_velocity = 0.00,q_steering_effort = 0.0,q_lat_accel = 0.005,q_obstacle_error_G = 1000000000.0, q_x_accel = 0.0, q_cruise_speed = 0.01, gas_max = 1.0, brake_max = 0.5):

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

    def calcOptimal(self,carnow,deernow,setSpeed = 25.0,yroad = 0.0):

        MPCF = MPC_F(q_lane_error = self.q_lane_error,q_obstacle_error = self.q_obstacle_error_F,q_lateral_velocity = self.q_lateral_velocity,q_steering_effort = self.q_obstacle_error_F,q_accel = self.q_lat_accel)
        MPCG = MPC_G(q_obstacle_error = self.q_obstacle_error_G, q_x_accel = self.q_x_accel,q_cruise_speed = self.q_cruise_speed, gas_max = self.gas_max, brake_max = self.brake_max)

        opt_steer = MPCF.calcOptimal(carnow, deernow, yroad)
        opt_gas,opt_brake = MPCG.calcOptimal(carnow, deernow, setSpeed)

        return opt_gas,opt_brake,opt_steer

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

    MPC = MPC_H(q_lane_error = 10.0,q_obstacle_error_F = 5000000.0,q_lateral_velocity = 0.00,q_steering_effort = 0.0,q_lat_accel = 0.005,q_obstacle_error_G = 1000000000.0, q_x_accel = 0.0, q_cruise_speed = 0.01, gas_max = 1.0, brake_max = 0.5)

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
            gas,brake,steer = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

            if ((sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2) < x_acceldistance) and (deer.x_Deer>car.x[2])):

                carx[k,:],carxdot[k,:] = car.heuns_update(gas = gas, brake = brake, steer = steer, cruise = 'off')
                deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]
                print "mpc active"

            else:
                carx[k,:],carxdot[k,:] = car.heuns_update(steer = steer, setspeed = setSpeed,)
                #carx[k,:],carxdot[k,:] = car.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
                deerx[k,:] = deer.updateDeer(car.x[2])
                #steervec[k] = opt_steer
                accelvec[k] = carxdot[k,3]

            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)
            print round(t[k],2),round(gas,2),round(brake,2)

    ayg = (carxdot[:,1]+carx[:,5]*carx[:,3])/9.81

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
    show()

if __name__ == '__main__':
    demo()