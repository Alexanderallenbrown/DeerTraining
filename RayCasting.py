import sys
from numpy import *
from sympy import *


def distanceray(pos):
    

    P = [-1,1][1,1][1,-1][-1,-1]

    angle = 3.1416/2

    x = Symbol('x')

    seg = 1

    yseg = (P[seg,1]-P[seg-1,1])/(P[seg,0]-P[seg-1,0])*(x-P[seg-1,0])+P[seg-1,1]
    yray = tan(angle)*(x-pos[1])+pos[2]

    xint = solve(yseg==yray)

    print xint

    yint = (P[seg,1]-P[seg-1,1])/(P[seg,0]-P[seg-1,0])*(xint-P[seg-1,0])+P[seg-1,1]

    if ((xint > P[seg-1,0]) and (xint < P[seg,0])):

        dist = sqrt((yint-pos[1])^2+(xint-pos[0])^2)

    else:

        print "don't intersect"

if __name__=='__main__':

    distanceray()