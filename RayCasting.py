import sys
from numpy import *
from matplotlib.pyplot import *
from KMLtoXYZ import *

def distanceray(pos, array):
    
    # Points of array (Begin and end with the same point)

    #print 'array', array
    P = double(array)

    dist_angle = zeros(360)

    for angle_deg in range(0,360):

        angle = angle_deg/180.0*3.1416

        dist_vec = zeros(len(P[0])-1)

        for seg in range(1,len(P[0])):

            # If segment is perpendicular:
            if (P[0,seg]==P[0,seg-1]):

                xint = P[0,seg]

                yint = tan(angle)*(xint-pos[0])+pos[1]

                #print seg    

            # If segment is not perpendicular:
            else:

                dydx_seg = ((P[1,seg]-P[1,seg-1])/(P[0,seg]-P[0,seg-1]))

                xint = (dydx_seg*P[0,seg-1]-P[1,seg-1]-tan(angle)*pos[0]+pos[1])/(dydx_seg-tan(angle))

                yint = (P[1,seg]-P[1,seg-1])/(P[0,seg]-P[0,seg-1])*(xint-P[0,seg-1])+P[1,seg-1]

            #print xint, yint

            # Check that the ray intercepts the segment
            if ((((xint >= P[0,seg-1]) and (xint <= P[0,seg])) or ((xint >= P[0,seg]) and (xint <= P[0,seg-1]))) and (((yint >= P[1,seg-1]) and (yint <= P[1,seg])) or ((yint >= P[1,seg]) and (yint <= P[1,seg-1])))):
                dist_vec[seg-1] = sqrt((yint-pos[1])**2+(xint-pos[0])**2)

            else:

                #print 'Out of segment Range'

                dist_vec[seg-1] = NaN

            # Check that the interception is for positive ray
            if (angle_deg>=90 and angle_deg<270):
                if (xint>pos[0]):

                    #print 'Negative Ray'
                    dist_vec[seg-1] = NaN

            else:
                if (xint<pos[0]):
                    dist_vec[seg-1] = NaN


            #print dist_vec

        dist_angle[angle_deg] = nanmin(dist_vec)



        if (dist_angle[angle_deg]>100):
            dist_angle[angle_deg] = 100

    #print dist_angle

    angle_plot = linspace(0,360,360)

    # figure()
    # plot(angle_plot,dist_angle)
    # title('Ray Casting')
    # xlabel('Angle (deg)')
    # ylabel('Minimum Distance (m)')

    #show()

    return dist_angle

def ProbDist(pos, psi1, mapa, q_dist = 1.0,q_dangle = 1/1000.0):

    distAngle=MapRaycasting(pos, mapa)

    distAngle_inv = 1/distAngle

    distAngle_inv = 1000.0*distAngle_inv - min(distAngle_inv)

    Prob1 = (distAngle_inv)/sum(distAngle_inv)

    # deltaPsi = zeros(360)

    # for k in range(0,359):
    #     deltaPsi[k]=abs(k-psi1)

    #     if deltaPsi[k]>180:
    #         deltaPsi[k] = 360-deltaPsi[k]

    # Prob = q_dangle*(180-deltaPsi)*q_dist*distAngle

    # Prob1 = Prob/sum(Prob)

    angle_plot = linspace(0,360,360)


    # figure()
    # plot(angle_plot,Prob1)
    # title('Probability Distribution')
    # xlabel('Angle (deg)')
    # ylabel('Probability')  

    #show() 

    return Prob1

def MapRaycasting(pos, mapa):

    sight_dist = 60.0

    f = mapa

    Trees = KMLtoXYZ(f)

    MinDist = []

    for ind in range(0,len(Trees)):
        MinDist.append(distanceray(pos, Trees[ind]))

    MinDist_Vec = zeros(len(MinDist[0]))

    d = zeros(len(MinDist))

    for ind in range(0,len(MinDist[0])):
        for ind1 in range(0,len(MinDist)):
            d[ind1] = MinDist[ind1][ind]

        MinDist_Vec[ind] = nanmin(d) 

        if isnan(MinDist_Vec[ind]):
            MinDist_Vec[ind] = sight_dist
        if MinDist_Vec[ind]>sight_dist:
            MinDist_Vec[ind] = sight_dist

    #print MinDist_Vec


    angle_plot = linspace(0,360,360)
    #hello
    # figure()
    # plot(angle_plot,MinDist_Vec)
    # title('Ray Casting')
    # xlabel('Angle (deg)')
    # ylabel('Minimum Distance (m)')

    # figure(facecolor='w')
    # subplot(111,facecolor='w')
    # seg = zeros((2,2))
    # print seg
    # for ind in range(0,360):
    #     seg[0][0] = pos[0]
    #     seg[0][1] = pos[0]+MinDist_Vec[ind]*cos(ind*3.1416/180)
    #     seg[1][0] = pos[1]
    #     seg[1][1] = pos[1]+MinDist_Vec[ind]*sin(ind*3.1416/180)
    #     plot(seg[0][:],seg[1][:],'k')
    # for ind in range(0,len(Trees)):
    #     plot(Trees[ind][0],Trees[ind][1],'g',linewidth = 2)
    #     fill(Trees[ind][0],Trees[ind][1],'g') 
    # plot(pos[0],pos[1],'ro')
    # axis('equal')
    # xlabel('X (m)', color='k')
    # ylabel('Y (m)', color='k')

    #show()

    return MinDist_Vec



if __name__=='__main__':

    #min_dist = MapRaycasting([200.0,0.0])

    dist = ProbDist([200.0,0.0],0.0, 'Test2.kml')

    angles = linspace(0,360,360)

    figure()
    plot(angles,dist)

    angle = zeros(10000)

    for k in range(0,10000):
        angle[k] = random.choice(len(dist),1,p =dist)

    figure()
    hist(angle, bins = 360)

    show()

    print dist

