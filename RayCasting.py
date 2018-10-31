import sys
from numpy import *
from matplotlib.pyplot import *

def distanceray(pos):
    
    # Points of array (Begin and end with the same point)
    P = array([[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0],[-1.0,1.0]])

    dist_angle = zeros(360)

    for angle_deg in range(0,359):

        angle = angle_deg/180.0*3.1416

        dist_vec = zeros(len(P)-1)

        for seg in range(1,len(P)):

            # If segment is perpendicular:
            if (P[seg,0]==P[seg-1,0]):

                xint = P[seg,0]

                yint = tan(angle)*(xint-pos[0])+pos[1]

                print seg    

                print xint, yint

            # If segment is not perpendicular:
            else:

                dydx_seg = abs((P[seg,1]-P[seg-1,1])/(P[seg,0]-P[seg-1,0]))

                print 'dydx', dydx_seg

                xint = (dydx_seg*P[0,0]-P[0,1]-tan(angle)*pos[0]+pos[1])/(dydx_seg-tan(angle))

                yint = (P[seg,1]-P[seg-1,1])/(P[seg,0]-P[seg-1,0])*(xint-P[seg-1,0])+P[seg-1,1]

                print seg

                print xint, yint

            # Check that the ray intercepts the segment
            if ((((xint >= P[seg-1,0]) and (xint <= P[seg,0])) or ((xint >= P[seg,0]) and (xint <= P[seg-1,0]))) and (((yint >= P[seg-1,1]) and (yint <= P[seg,1])) or ((yint >= P[seg,1]) and (yint <= P[seg-1,1])))):
                dist_vec[seg-1] = sqrt((yint-pos[1])**2+(xint-pos[0])**2)



                print yint-pos[1]

                print xint-pos[0]

                print dist_vec[seg-1]

            else:

                print 'Out of segment Range'

                dist_vec[seg-1] = NaN

            # Check that the interception is for positive ray
            # if (angle_deg>90 and angle_deg<270):

            #     if (xint>pos[0]):

            #         print 'Negative Ray'
            #         dist_vec[seg-1] = NaN

            #else:

                #print 'nan'

               # if (xint<pos[0]):
                    #dist_vec[seg-1] = NaN

        dist_angle[angle_deg] = nanmin(dist_vec)

        if (dist_angle[angle_deg]>100):
            dist_angle[angle_deg] = 100

    print dist_angle

    angle_plot = linspace(0,360,360)

    figure()
    plot(angle_plot,dist_angle)
    title('Ray Casting')
    xlabel('Angle (deg)')
    ylabel('Minimum Distance (m)')

    #show()

    return dist_angle

def ProbDist(pos, psi1, q_dist = 1,q_dangle = 1/10):

    distAngle=distanceray(pos)

    deltaPsi = zeros(360)

    for k in range(0,359):
        deltaPsi[k]=abs(k-psi1)

        if deltaPsi[k]>180:
            deltaPsi[k] = 360-deltaPsi[k]

    Prob = q_dangle*deltaPsi*q_dist*distAngle

    Prob1 = Prob/sum(Prob)

    angle_plot = linspace(0,360,360)

    figure()
    plot(angle_plot,Prob1)
    title('Probability Distribution')
    xlabel('Angle (deg)')
    ylabel('Probability')  

    show() 





if __name__=='__main__':

    ProbDist([0,0],0)