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

bbody = zencad.cylinder(r=5, h=L, center=True).rotateY(deg(90)).moveY(40)
abbody = zencad.disp(bbody)

a = rigid_body(inertia=inertia(radius=pyservoce.vector3(10,0,0)), pose=zencad.transform.nulltrans())
a.add_view(abody)

b = rigid_body(inertia=inertia(radius=pyservoce.vector3(0,0,0)), pose=zencad.moveX(10))
b.add_view(abbody)

c = constraits.spherical_rotator()
c.attach_reference(body=a, pose=left(0))

c2 = constraits.spherical_rotator()
c2.attach_reference(body=b, pose=left(10))

#a.set_speed(screw(lin=(0,0,0), ang=(200,20,0)))

solver = zencad.elibs.solver.matrix_solver(rigid_bodies=[a], constraits=[c], 
	world_dempher=0.1, gravity=(0,0,-9.81))
solver.update_views()
solver.update_globals()

solver2 = zencad.elibs.solver.matrix_solver(rigid_bodies=[b], constraits=[c2], 
	world_dempher=0.1, gravity=(0,0,-9.81))
solver2.update_views()
solver2.update_globals()

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

	maxdelta = 0.01
	curtime = time.time()
	delta = curtime - lasttime
	lasttime = curtime

	if delta > maxdelta:
		delta = maxdelta
	DELTA = delta

	#print(solver.constrait_matrix()[0]

	solver.solve()
	solver.apply(DELTA)
	solver2.solve()
	solver2.apply(DELTA)
	#print(solver.A)
	#print(solver.B)

	#print(solver.accelerations)
	#exit(0)

show(animate=animate, animate_step = 0.00001)