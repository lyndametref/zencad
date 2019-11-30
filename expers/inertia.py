#!/usr/bin/env python3

from zencad import *
import zencad.libs.inertia as inert

import numpy

a = inert.inertia(
	mass=1, 
	matrix=numpy.matrix('1 0 0; 0 1 0; 0 0 1'),
	cm=(0,0,1)
)

b = inert.inertia(
	mass=10, 
	matrix=numpy.matrix('1 0 0; 0 1 0; 0 0 1'),
	cm=(2,1,20)
)

#print(a)
#print(a.transform(translate(0,0,1) * rotateZ(deg(90))))

print(a)
print(b)
print(inert.complex_inertia([a,b]))