import sys
from numpy import *
from BicycleModel import *
from matplotlib.pyplot import *
from Deer import *

def BinaryConversion(ind):

	resolution = 5

	# Set minimum and maximum values

	Psi0_min = -3.14/2 # radians
	Psi0_max = 3.14/2 # radians

	SigmaPsi_min = 0 # radians
	SigmaPsi_max = 0.45*3.24 # radians

	tturn_min = 0.4 # seconds
	tturn_max = 2.5 # seconds

	Vmax_min = 5 # m/s
	Vmax_max = 18 # m/s

	Tau_min = 0.75 # seconds
	Tau_max = 5 # seconds

	# Divide individual into different binary 
	Psi0_bin = ind[0:resolution]
	SigmaPsi_bin = ind[resolution:2*resolution]
	tturn_bin = ind[2*resolution:3*resolution]
	Vmax_bin = ind[3*resolution:4*resolution]
	Tau_bin = ind[4*resolution:5*resolution]

	# Convert from binary to decimala
	Psi0 = Psi0_min + (Psi0_max - Psi0_min)*float(int(Psi0_bin,2))/((2**resolution)-1)
	SigmaPsi = SigmaPsi_min + (SigmaPsi_max - SigmaPsi_min)*float(int(SigmaPsi_bin,2))/((2**resolution)-1)
	tturn = tturn_min + (tturn_max - tturn_min)*float(int(tturn_bin,2))/((2**resolution)-1)
	Vmax = Vmax_min + (Vmax_max - Vmax_min)*float(int(Vmax_bin,2))/((2**resolution)-1)
	Tau = Tau_min + (Tau_max - Tau_min)*float(int(Tau_bin,2))/((2**resolution)-1)

	#Rrint results
	# print(Psi0)
	# print(SigmaPsi)
	# print(tturn)
	# print(Vmax)
	# print(Tau)

	return array([Psi0,SigmaPsi,tturn,Vmax,Tau])

def TestDeer(deer_ind, n):

	min_distance = zeros(n)

	for k_1 in range(0,n):

		deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

		# Indicate deer initial position
	 	deer.x_Deer = 80
	 	deer.y_Deer = -2
	 	# Define simulation time and dt
	 	simtime = 10
	 	dt = deer.dT
	 	t = arange(0,simtime,dt) #takes min, max, and timestep\

	    #now set up the car's parameters
	 	car = BicycleModel(dT=dt,U=20)
	 	steervec = zeros(len(t))

	    #initialize matrices to hold simulation data
	    #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
	 	carx = zeros((len(t),len(car.x)))
	 	carx[0,:] = car.x

	    #initialize for deer as well
	 	deerx = zeros((len(t),4))
	    #fill in initial conditions because they're nonzero
	 	deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

	    #now simulate!!
	 	for k in range(1,len(t)):
	 	 	carx[k,:],junk=car.euler_update(steervec[k],autopilot='on')
	 		deerx[k,:] = deer.updateDeer(car.x[2])

	 	distance = sqrt((carx[:,2]-deerx[:,2])**2+(carx[:,0]-deerx[:,3])**2)

	 	# print(min(distance))

	 	min_distance[k_1] = min(distance)

		
		# print(min_distance)

	# Calculate IQM

	# Sort values from smallest to largest
	min_distance = sorted(min_distance)
	# Eliminate lower and upper quartiles
	min_distance = min_distance[2:6]
	# Calculate the IQM
	avg_min_distance = mean(min_distance)
	# print(avg_min_distance)

	return(avg_min_distance)

if __name__=='__main__':

	Deer1 = BinaryConversion('0101010000110001110001001')

	print(Deer1)

	Distance1 = TestDeer(deer_ind=Deer1, n=8)

	print(Distance1)
