#!/usr/bin/env python3

import numpy
import pyservoce
import zencad
import time
from zencad import *

import zencad.mbody.solver
import zencad.mbody.constraits as constraits
from zencad.mbody.rigid_body import rigid_body
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

a.set_speed(screw(lin=(0,0,0), ang=(2,2,2)))

solver = zencad.mbody.solver.matrix_solver(rigid_bodies=[a], constraits=[])
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

	#print(solver.constrait_matrix()[0])
	#print(solver.inertia_forces())


	#print("M")
	print(solver.mass_matrix())

	solver.solve()
	solver.apply(DELTA)

show(animate=animate, animate_step = 0.00001)