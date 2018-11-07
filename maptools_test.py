from maptools import *


if __name__=='__main__':
    c = open('LatLongTest1.csv')

    print c

    mapa = Map(filename='LatLongTest1.csv')

    c1 = mapa.loadCSVXYZMap('LatLongTest1.csv')

    print 'c1', c1