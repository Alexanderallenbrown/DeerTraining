import sys
from numpy import *
from BicycleModel import *
from matplotlib.pyplot import *
from Deer import *

if __name__=='__main__':

	print('Start')
	# Create minumum distance vector
	generationtimes = zeros(15)
	#generation[:,0] = (1,2,3,4,5,6,7,8,9,10,11,12,13,14,15)
	#print(generation)

	for deerN in range(0,3):
		min_distance = zeros(8)

		# Simulate 8 trials for each deer
		for k_1 in range(0,8):

			deer = Deer()
		 	deer.x_Deer = 80
		 	deer.y_Deer = -2

		 	simtime = 10
		 	dt = deer.dT
		 	t = arange(0,simtime,dt) #takes min, max, and timestep

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

		 	#print(min(distance))
		 	min_distance[k_1] = min(distance)

		    # #now plot stuff
		 	
		 	# figure()
		 	# plot(carx[:,2],carx[:,0],'ko',deerx[:,2],deerx[:,3],'ro')

		 	# figure()
		 	# subplot(2,1,1)
		 	# plot(t,deerx[:,2])
		 	# subplot(2,1,2)
		 	# plot(t,deerx[:,3])

		 	# deermoving_index = nonzero((deerx[:,2]-carx[:,2])<=deer.x_StartDeer)
		 	# deermoving_index = deermoving_index[0][0]-1
		 	# turn_index = nonzero(t>=(deer.tturn_Deer+t[deermoving_index]))
		 	# turn_index = turn_index[0][0]

		 	# figure()
		 	# subplot(2,1,1)
		 	# plot(t,deerx[:,0],t[deermoving_index],deerx[deermoving_index,0],'ro',t[turn_index],deerx[turn_index,0],'bo')
		 	# subplot(2,1,2)
		 	# plot(t,carx[:,2],t[deermoving_index],carx[deermoving_index,2],'ro')

		 	# print(deer.Psi0_Deer,deer.Psi1_Deer,deer.Psi2_Deer)
		 	# print(deer.Vturn_Deer,deer.Vmax_Deer)

		 	# figure()
		 	# plot(t,distance)


		 	#show()

		print(min_distance)

		# Calculate IQM

		# Sort values from smallest to largest
		min_distance = sorted(min_distance)
		# Eliminate lower and upper quartiles
		min_distance = min_distance[2:6]
		# Calculate the IQM
		avg_min_distance = mean(min_distance)
		print(avg_min_distance)

		generationtimes[deerN] = avg_min_distance
		print(generationtimes)

	indtimes = argsort(generationtimes)
	print("indtimes")
	print(indtimes)
