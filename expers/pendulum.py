#!/usr/bin/env python3

import zencad
from zencad import *
import time
import zencad.libs.treedy as treedy
import zencad.libs.forces as forces
import zencad.libs.inertia as inertia
from zencad.libs.screw import screw
import zencad.mbody.kinematic as kinematic

import numpy

numpy.set_printoptions(precision=5, linewidth=200)
numpy.set_printoptions(suppress=True)
#numpy.set_printoptions(precision=1, linewidth=160)

L = 100
base = zencad.assemble.unit()
arot = kinematic.rotator(name="AROT",ax=(1,0,0))
a = zencad.assemble.unit(name="A")
ma = zencad.assemble.unit(name="MA", shape=sphere(10))

a.add_shape(cylinder(r=5, h=L))

base.relocate(rotateY(deg(90)))
base.link(arot)
arot.link(a)
a.link(ma)

a.link(ma)
ma.relocate(up(L))

inertia.attach_inertia(ma, mass=1, Ix=1, Iy=1, Iz=1)

arot.set_speed(-0.2)
t = treedy.tree_dynamic_solver(base)

cancel = False
DELTA = 0

def evaluate():
	global DELTA
	starttime = time.time()
	lasttime = starttime

	time.sleep(2)

	#cp.enable()
	while True:
		if cancel:
			return
		maxdelta = 20
		curtime = time.time()
		delta = curtime - lasttime
		lasttime = curtime


		if delta > maxdelta:
			delta = maxdelta
		DELTA = delta

		t.onestep(delta=delta)

		print(t.reaction_solver.mass_matrix())
		print(t.reaction_solver.reactions)

		#print(t.reaction_solver.inertia_forces())
		time.sleep(0.01)

def animate(self):
	base.location_update(deep=True, view=True)

import threading
evalthr = threading.Thread(target=evaluate)
evalthr.start()

def close_handle():
	global cancel
	cancel = True

disp(base)
show(animate=animate, animate_step = 0.00001, close_handle=close_handle)