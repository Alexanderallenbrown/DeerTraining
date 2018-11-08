from matplotlib.pyplot import *
from maptools import *

if __name__=='__main__':

    c = open('LatLongTest1.csv')

    Road = zeros((2,3))


    print c

    f = open('maptools_folder/RoadStart.csv')

    k = 0
    for line in f:
        stripline = line.strip()
        linesplit = line.split(',')
        Road[k][0] = float(linesplit[0])
        Road[k][1] = float(linesplit[1])
        Road[k][2] = float(linesplit[2])
        k+=1

    print Road

    mapRoad = Map(filename='RoadStart.csv')

    psi = (atan2(mapRoad.Y[1]-mapRoad.Y[0],mapRoad.X[1]-mapRoad.X[0]))

    print psi

    mapa = Map(filename='LatLongTest1.csv', ireflat = Road[0][0], ireflon = Road[0][1], irefelev = Road[0][2])

    print mapa.X
    print mapa.Y
    print mapa.Z

    XYZ = mapa.mapFrameToLocalFrame(mapa.X,mapa.Y,mapa.Z,yaw = psi)

    print XYZ

    #RoadEnd = mapa.mapFrameToLocalFrame(mapRoad.X[1],mapRoad.Y[1],mapRoad.Z[1], yaw = psi)

    #print RoadEnd

    figure()
    plot(mapa.X,mapa.Y,0,0,mapRoad.X,mapRoad.Y)
    axis('equal')

    figure()
    plot(XYZ[0],XYZ[1],0,0)
    axis('equal')

    show()





