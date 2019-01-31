import sys
from numpy import array,dot,cos,sin,sqrt,tan,sign,zeros,ones
from Cpacejkatire import PacejkaTire
cimport cython
# from cython.view import array as cvarray
#from matplotlib.pyplot import *

cdef class BicycleModel:
    cdef float a,b,m,I,U,Caf,Car,dT,mu,bias,autopilot_gainxinit,yinit,zinit
    cdef str tiretype,drive,steering_actuator
    cdef float[:] x_view, xdot_view

    def __init__(self,a = 0.8,b = 1.5,m = 1000.0,I=2000.0,U=20.0,Cf=100000.0,Cr=100000.0,mu=1.0,dT=0.01,tiretype='pacejka',drive='rear',bias=0.3,autopilot_gain = 1, xinit = 0, yinit = 0, zinit = 0,steering_actuator = 'on'):
        """ 
        BicycleModel(a = 1.2,b = 1.0,m = 1000.0,I=2000.0,U=20.0,Cf=-100000.0,Cr=-100000.0,mu=1.0,dT=0.01,tiretype='dugoff',coords='local'))
        This is a bicycle model. It can use a dugoff tire model or a linear tire model.

        Right now, there is only front/rear asymmetrical braking (no left/right bias) so steering with the brakes is not an option.

        Right now, very simple engine model with constant power. See euler_update for details. planning to break these out soon.

         """
        self.a = a
        self.b = b
        self.m = m
        self.I = I
        self.U = U
        self.Caf = Cf
        self.Car = Cr
        self.dT = dT
        #self.tire = DugoffTire()#pass arguments eventually
        self.mu = mu
        self.tiretype = tiretype
        self.drive = drive
        self.bias = bias
        self.Fyf = 0
        self.Fyr = 0
        self.Fxf = 0
        self.Fxr = 0
        self.alpha_f = 0
        self.alpha_r = 0
        self.autopilot_gain=autopilot_gain
        self.steer_limit = 0.25 #radioans
        self.cruise_gain = 5
        #calculate vertical tire forces TODO roll model? Longitudinal Weight Transfer?
        self.Fzf = self.m*self.b/(self.a+self.b)*9.81
        self.Fzr = self.m*self.a/(self.a+self.b)*9.81
        self.pacejkatire_front = PacejkaTire(self.Fzf/2,mu,mu)
        self.pacejkatire_rear = PacejkaTire(self.Fzf/2,mu,mu)

        # Steering actuator dynamics
        self.steering_actuator = steering_actuator
        self.z = 0.9
        self.w = 15
        self.delta_r = 0
        self.delta_rdot = 0
        self.delta_rold = 0
        self.e = 0
        self.e_old = 0
        self.delta = 0
        self.deltadot = 0
        self.deltaold = 0

        #engine stuff TODO:
        self.power  = 20000 #~100 HP  (100 KW) engine, constant power all the time. For now.

        #aero stuff: taken from a Nissan Leaf drag area 7.8 ft^2 0.725 m^2, Cd 0.32
        self.CdA = .32


        #state order is y,v,x,u,psi,r
        self.x = array([yinit,0,xinit,0,0,0],dtype=float)
        self.x_view = self.x
        self.xdot = zeros(6,dtype=float)
        self.xdot_view = self.xdot

    cdef float dugoffFy(self,alpha,Fz,mu=1.5,Fx=0,Ca=100000):
        cdef float mufz,lam,flam,Fy
        if abs(alpha)>0:
            mufz = sqrt((mu*Fz)**2-Fx**2)#De-rate lateral force capacity for braking
            lam= mufz/(2*Ca*abs(tan(alpha))) #lambda
            if lam>=1:
                flam = 1
            else:
                flam = (2-lam)*lam
            Fy = -Ca*tan(alpha)*flam
        else:
            Fy=0
        return Fy

    def state_eq(self,float[:] x,t,Fxf,Fxr,delta):
        cdef float Ydot,vdot,Xdot,Udot,Psidot,rdot
        """ state_eq(self,x,t,Fx,delta):
            returns state derivatives for odeint"""

        if abs(Fxf/self.Fzf)>self.mu: #make sure we can't apply any more brakes than makes sense.
            Fxf = self.Fzf*self.mu*sign(Fxf)
        if abs(Fxr/self.Fzr)>self.mu: #make sure we can't apply any more brakes than makes sense.
            Fxr = self.Fzr*self.mu*sign(Fxr)
        #calculate slip angles
        if abs(x[3])>0:
            self.alpha_f = 1/x[3]*(x[1]+self.a*x[5])-delta
            self.alpha_r = 1/x[3]*(x[1]-self.b*x[5]) 
            #print self.alpha_f,self.alpha_r
        else:#if the car is only spinning, but not driving forward, then alpha is nonsense
            self.alpha_f = 0
            self.alpha_r = 0

        #calculate tire forces
        if self.tiretype=='dugoff':
            #if the car is moving, use a real tire model
            if self.x[3]>0:
                self.Fyf = self.dugoffFy(self.alpha_f,self.Fzf,self.mu,Fxf,self.Caf)
                self.Fyr = self.dugoffFy(self.alpha_r,self.Fzr,self.mu,Fxr,self.Car)
            else:
                #the car isn't really moving, so use static friction
                self.Fyf = -self.Fzf*self.mu*sign(self.x[1]+self.a*self.x[5])
                self.Fyr = -self.Fzr*self.mu*sign(self.x[1]-self.b*self.x[5])
        elif self.tiretype=='pacejka':
            if self.x[3]>0:
                #calculate effective Calphas RIGHT NOW
                Fyf_high = 2*self.pacejkatire_front.calcFy_xforce(self.Fzf/2,self.alpha_f+.01,Fxf,0)
                Fyf_low = 2*self.pacejkatire_front.calcFy_xforce(self.Fzf/2,self.alpha_f-.01,Fxf,0)
                Fyr_high = 2*self.pacejkatire_rear.calcFy_xforce(self.Fzr/2,self.alpha_r+.01,Fxr,0)
                Fyr_low = 2*self.pacejkatire_rear.calcFy_xforce(self.Fzr/2,self.alpha_r-.01,Fxr,0)
                self.Caf = abs(Fyf_high-Fyf_low)/(.02)
                self.Car = abs(Fyr_high-Fyr_low)/(.02)

                self.Fyf = 2*self.pacejkatire_front.calcFy_xforce(self.Fzf/2,self.alpha_f,Fxf,0)
                self.Fyr = 2*self.pacejkatire_rear.calcFy_xforce(self.Fzr/2,self.alpha_r,Fxr,0)
            else:
                #the car isn't really moving, so use static friction
                self.Fyf = -self.Fzf*self.mu*sign(self.x[1]+self.a*self.x[5])
                self.Fyr = -self.Fzr*self.mu*sign(self.x[1]-self.b*self.x[5])
        else:
            if self.x[3]>0:
                self.Fyf = -self.alpha_f*self.Caf
                self.Fyr = -self.alpha_r*self.Car
            else:
                #the car isn't really moving, so use static friction
                self.Fyf = -self.Fzf*self.mu*sign(self.x[1]+self.a*self.x[5])
                self.Fyr = -self.Fzr*self.mu*sign(self.x[1]-self.b*self.x[5])

        Ydot = x[1]*cos(x[4])+x[3]*sin(x[4]) #north velocity NOT local!
        vdot = -x[3]*x[5] + 1/self.m*(self.Fyf+self.Fyr) #derivative of local lateral velocity
        Xdot = x[3]*cos(x[4])-x[1]*sin(x[4]) #East velocity NOT local!
        Udot = x[5]*x[1]+(Fxf+Fxr)/self.m #this is the derivative of local forward speed
        Psidot = x[5]
        rdot = (self.a*self.Fyf-self.b*self.Fyr)/self.I #cosines in there? Small angle? Sounds/Looks OK...
        #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
        return array([Ydot,vdot,Xdot,Udot,Psidot,rdot],dtype=float)

    def calc_inputs(self,brake,gas,steer):
        #first calculate engine power accepting an input from 0 to 1
        #gas = -gas
        cdef float Engine_Force,total_brake_force,front_brake_force,rear_brake_force,Fxf,Fxr
        if gas<0:
            gas = 0
        if abs(gas)>1:
            gas = 1
            
        if self.x[3]>0:
            Engine_Force = gas * self.power/self.x[3]
            if Engine_Force>self.mu*self.m*9.81:
                Engine_Force = self.mu*self.m*9.81
        else:
            Engine_Force = gas*self.mu*self.m*9.81
        
        if Engine_Force<0:
            Engine_Force = 0

            ############## ABS GOES HERE ###############
        #now calculate the brake forces accepting an input from 0 to 1
        total_brake_force = brake * (self.m*9.81) #max brakes out. TODO model brake fade and speed dependence
        front_brake_force = total_brake_force*(1-self.bias)#so the brake bias of 1 means all rear brakes
        rear_brake_force = total_brake_force*self.bias

        if front_brake_force > (self.m*9.81*self.mu*self.b/(self.a+self.b)):
            front_brake_force = self.m*9.81*self.mu*self.b/(self.a+self.b)

        if rear_brake_force > (self.m*9.81*self.mu*self.a/(self.a+self.b)):
            rear_brake_force = self.m*9.81*self.mu*self.a/(self.a+self.b)

        if self.U>0:
            if self.drive=='rear':
                Fxf = -front_brake_force
                Fxr = -rear_brake_force +Engine_Force
            elif self.drive=='all':
                Fxf = -front_brake_force+Engine_Force/2
                Fxr = -rear_brake_force+Engine_Force/2
            else:
                Fxf = -front_brake_force+Engine_Force
                Fxr = -rear_brake_force
        else:
            if self.drive=='rear':
                Fxf = -front_brake_force*sign(self.x[3])
                Fxr = -rear_brake_force*sign(self.x[3]) +Engine_Force
            elif self.drive=='all':
                Fxf = -front_brake_force*sign(self.x[3])+Engine_Force/2
                Fxr = -rear_brake_force*sign(self.x[3])+Engine_Force/2
            else:
                Fxf = -front_brake_force*sign(self.x[3])+Engine_Force
                Fxr = -rear_brake_force*sign(self.x[3])
        
        return Fxf,Fxr,steer

    def cruise(self,setspeed):
        gas = self.cruise_gain*(setspeed-self.U)#Just p-control for the moment TODO
        
        if gas<0:
            gas = 0
        return gas

    def ACC(self,setspeed,obsloc,obsvel):
        #this is for adaptive cruise control. NOT TESTED PROB DOESNT WORK. ASSUMES obstacle is directly in front.
        #first set nominal gas
        gas = self.cruise(setspeed)
        #now modify gas for obstacle
        distance_to_obs = obsloc-self.x
        if distance_to_obs>0: #if the obstacle is actually in front of us
            pass


    def euler_update(self,brake=0,gas=0,steer=0,cruise = 'on',setspeed=20.0,autopilot='off',mpc='off',patherror=0):
        cdef float Fxf,Fxr
        Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        if cruise=='on': #TODO make this better. Hardcoded and awful for now...
            gas = self.cruise(setspeed)
            Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        if autopilot=='on':
            steer = self.autopilot_gain*patherror #VERY simple autopilot. Path error can be previewed, or can be angle, or whatever.
            if abs(steer)>self.steer_limit:
                steer = sign(steer)*self.steer_limit
            #print steer
            Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        

        #this should have taken casre of all contingencies and got us the correct inputs for the car.
        self.xdot = self.state_eq(self.x,0,Fxf,Fxr,steer)
        self.x = self.x+self.dT*self.xdot #this should update the states
        self.U = self.x[3]
        #print("actual inputs passed:     "+str(Fxf)+","+str(Fxr)+","+str(steer))
        return self.x,self.xdot

    def heuns_update(self,brake=0,gas=0,steer=0,cruise = 'on',setspeed=20.0,autopilot='off',mpc='off',patherror=0):

        # Update steering actuator
        if self.steering_actuator == 'on':
            steer = self.steering_update(steer)

        Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        if cruise=='on': #TODO make this better. Hardcoded and awful for now...
            gas = self.cruise(setspeed)
            Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        if autopilot=='on':
            steer = self.autopilot_gain*patherror #VERY simple autopilot. Path error can be previewed, or can be angle, or whatever.
            if abs(steer)>self.steer_limit:
                steer = sign(steer)*self.steer_limit
            #print steer
            Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)

        #this should have taken case of all contingencies and got us the correct inputs for the car.
        k1x = self.state_eq(self.x,0,Fxf,Fxr,steer) # Calvulate k1
        xhat = self.x + self.dT*k1x # Find x_hat
        k2x = self.state_eq(xhat,0,Fxf,Fxr,steer) # Calcaulte k2 using x_hat
        self.xdot = (k1x+k2x)/2 # Find xdot by averaging k1 and k2

        self.x = self.x + self.xdot*self.dT
        self.U = self.x[3]

        return self.x,self.xdot,steer

    def rk_update(self,brake=0,gas=0,steer=0,cruise = 'on',setspeed=20.0,autopilot='off',mpc='off',patherror=0):
       
        # Update steering actuator
        if self.steering_actuator == 'on':
            steer = self.steering_update(steer)

        Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        if cruise=='on': #TODO make this better. Hardcoded and awful for now...
            gas = self.cruise(setspeed)
            Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        if autopilot=='on':
            steer = self.autopilot_gain*patherror #VERY simple autopilot. Path error can be previewed, or can be angle, or whatever.
            if abs(steer)>self.steer_limit:
                steer = sign(steer)*self.steer_limit
            #print steer
            Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)

        #this should have taken case of all contingencies and got us the correct inputs for the car.
        #cdef float[:] k1x,xhat1,k2x,xhat2,k3x,xhat3,k4x
        k1x = self.state_eq(self.x,0,Fxf,Fxr,steer) # Calvulate k1
        xhat1 = self.x + self.dT*k1x/2 # Find x_hat1
        k2x = self.state_eq(xhat1,0,Fxf,Fxr,steer) # Calcaulte k2 using x_hat1
        xhat2 = self.x + self.dT*k2x/2 # Find x_hat2
        k3x = self.state_eq(xhat2,0,Fxf,Fxr,steer) # Calcaulte k3 using x_hat2
        xhat3 = self.x + self.dT*k3x # Find x_hat3
        k4x = self.state_eq(xhat3,0,Fxf,Fxr,steer) # Calcaulte k4 using x_hat3

        # Calculate xdot
        self.xdot_view = (k1x+2.0*k2x+2.0*k3x+k4x)/6.0 # Find xdot by averaging k1 and k2

        self.x_view = self.x + self.xdot*self.dT
        self.U = self.x_view[3]

        return self.x,self.xdot,steer


    def euler_predict(self,x,brake=0,gas=0,steer=0,dT=0.1):
        Fxf,Fxr,steer = self.calc_inputs(brake,gas,steer)
        self.xdot = self.state_eq(x,0,Fxf,Fxr,steer)
        x = x+self.dT*self.xdot #this should update the states
        return x,self.xdot


    def steering_statederivs(self, delta, deltadot, delta_r, delta_rdot):
        deltaddot = 2 * self.z * self.w * (delta_rdot - deltadot) + self.w * self.w * (delta_r - delta)
        return [deltadot, deltaddot]
  
    def steering_update(self, delta_r):
        self.delta_r = delta_r;
        self.delta_rdot = (self.delta_r - self.delta_rold) / self.dT;
        self.deltadot = (self.delta - self.deltaold) / self.dT;
        self.deltaold = self.delta;
        self.delta_rold = self.delta_r;
        #self uses Heun's method (trapezoidal)
        xdot1 = self.steering_statederivs(self.delta, self.deltadot, self.delta_r, self.delta_rdot);
        #first calculation
        deltaprime = self.delta + xdot1[0] * self.dT;
        deltadotprime = self.deltadot + xdot1[1] * self.dT;
        #now compute again
        xdot2 = self.steering_statederivs(deltaprime, deltadotprime, self.delta_r, self.delta_rdot);
        #now compute the final update
        self.delta = self.delta + self.dT / 2 * (xdot1[0] + xdot2[0]);
        self.deltadot = self.deltadot + self.dT / 2 * (xdot1[1] + xdot2[1]);

        if (self.delta > self.steer_limit):
            self.delta = self.steer_limit

        if (self.delta < -self.steer_limit):
            self.delta = -self.steer_limit

        return self.delta;
