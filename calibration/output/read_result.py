import cv
import numpy as np

fp = 'calib_result.yml'
ma = np.asarray( cv.Load( fp, cv.CreateMemStorage(), 'Marker Position' ) )
mb = np.asarray( cv.Load( fp, cv.CreateMemStorage(), 'Kinect Angle' ) )
mc = np.asarray( cv.Load( fp, cv.CreateMemStorage(), 'Bar Centers' ) )
print 'Marker Position:'
print ma
print 'Marker_0:', ma[0]
print 'Kinect Angle:', mb[0]
print 'Bar Centers:'
print mc
print 'Bar_Centers_0:', mc[0]
