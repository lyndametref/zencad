#!/usr/bin/env python3

import numpy
import pyservoce
import zencad
import time
from zencad import *

import zencad.elibs.solver
import zencad.elibs.constraits as constraits
from zencad.elibs.rigid_body import rigid_body
#import zencad.mbody.kinematic as kinematic
from zencad.libs.inertia import inertia
from zencad.libs.screw import screw
numpy.set_printoptions(suppress=True)
numpy.set_printoptions(precision=5, linewidth=160)

L=20

body = zencad.cylinder(r=5, h=L).rotateY(deg(90))
abody = zencad.disp(body)

a = rigid_body(inertia=inertia(radius=pyservoce.vector3(10,0,0)), pose=zencad.transform.nulltrans())
a.add_view(abody)

c = constraits.spherical_rotator()
c.attach_reference(body=a, pose=nulltrans())

a.set_speed(screw(lin=(0,0,0), ang=(200,20,0)))

solver = zencad.elibs.solver.matrix_solver(rigid_bodies=[a], constraits=[c])
solver.update_views()
solver.update_globals()

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

	#print(solver.constrait_matrix()[0])
	print(solver.inertia_forces())

	solver.solve()
	solver.apply(DELTA)

show(animate=animate, animate_step = 0.00001)