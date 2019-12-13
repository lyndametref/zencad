#!/usr/bin/env python3

import numpy
import pyservoce
import zencad
import time
from zencad import *

import zencad.elibs.solver
import zencad.elibs.constraits as constraits
from zencad.elibs.rigid_body import rigid_body
#import zencad.libs.kinematic as kinematic
from zencad.libs.inertia import inertia
from zencad.libs.screw import screw
numpy.set_printoptions(suppress=True)
numpy.set_printoptions(precision=3, linewidth=160)

L=20

body = zencad.cylinder(r=5, h=L).rotateY(deg(90))
body2 = zencad.cylinder(r=5, h=L).rotateY(deg(90))
abody = zencad.disp(body)
bbody = zencad.disp(body2)

a = rigid_body(inertia=inertia(radius = pyservoce.vector3(10,0,0)), pose=zencad.transform.nulltrans())
b = rigid_body(inertia=inertia(radius = pyservoce.vector3(10,0,0)), pose=zencad.transform.right(20))
a.add_view(abody)
b.add_view(bbody)

#b.pose=zencad.transform.right(20) #* zencad.transform.rotateY(deg(20))
a.set_speed(screw(lin=(0,0,0), ang=(0,0,2)))
b.set_speed(screw(lin=(0,2*20,0), ang=(0,0,3)))

c = constraits.spherical_rotator()
c.attach_positive_connection(body=b, radius=pyservoce.vector3(0,0,0))
c.attach_negative_connection(body=a, radius=pyservoce.vector3(20,0,0))

#c1 = constraits.spherical_rotator()
#c1.attach_positive_connection(body=a, radius=pyservoce.vector3(0,0,0))

solver = zencad.elibs.solver.matrix_solver(rigid_bodies=[a,b], constraits=[c])
solver.update_views()
solver.update_globals()

print("mass matrix")
print(solver.mass_matrix())
print("constrait matrix")
print(solver.constrait_matrix()[0])
print("inertia_forces")
print(solver.inertia_forces())

accs, react = solver.solve()

print("acc")
print(accs)
print("react")
print(react)

#exit(0)
starttime = time.time()
lasttime = starttime
noinited = True
def animate(wdg):
	global noinited
	global lasttime

	if noinited:
		time.sleep(1)
		noinited= False

	maxdelta = 0.001
	curtime = time.time()
	delta = curtime - lasttime
	lasttime = curtime

	if delta > maxdelta:
		delta = maxdelta
	DELTA = delta

	solver.solve()
	solver.apply(DELTA)

show(animate=animate, animate_step = 0.00001)