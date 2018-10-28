import sys
from numpy import *
from matplotlib.pyplot import *

def distanceray(pos):
    
    # Points of array (Begin and end with the same point)
    P = array([[-1,1],[1,1],[1,-1],[-1,-1],[-1,1]])

    dist_angle = zeros(359)

    for angle_deg in range(0,359):

        angle = angle_deg/180.0*3.1416

        dist_vec = zeros(len(P)-1)

        for seg in range(1,len(P)):

            # If segment is perpendicular:
            if (P[seg,0]==P[seg-1,0]):

                xint = P[seg,0]

                yint = tan(angle)*(xint-pos[0])+pos[1]

            # If segment is not perpendicular:
            else:

                dydx_seg = (P[seg,1]-P[seg-1,1])/(P[seg,0]-P[seg-1,0])

                xint = (dydx_seg*P[0,0]-P[0,1]-tan(angle)*pos[0]+pos[1])/(dydx_seg-tan(angle))

                yint = (P[seg,1]-P[seg-1,1])/(P[seg,0]-P[seg-1,0])*(xint-P[seg-1,0])+P[seg-1,1]


            # Check that the ray intercepts the segment
            if ((xint >= P[seg-1,0]) and (xint <= P[seg,0])):
                dist_vec[seg-1] = sqrt((yint-pos[1])**2+(xint-pos[0])**2)

            else:
                dist_vec[seg-1] = NaN

            # Check that the interception is for positive ray
            if (angle_deg>90 and angle_deg<270):
                if (xint>pos[0]):
                    dist_vec[seg-1] = NaN

            else:
                if (xint<pos[0]):
                    dist_vec[seg-1] = NaN

        dist_angle[angle_deg] = nanmin(dist_vec)

    #print dist_angle

    angle_plot = linspace(0,359,359)

    figure()
    plot(angle_plot,dist_angle)

if __name__=='__main__':

    distanceray([0,0.75])