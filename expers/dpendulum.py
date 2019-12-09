#!/usr/bin/env python3

import zencad
from zencad import *
import time
import zencad.libs.treedy as treedy
import zencad.libs.forces as forces
import zencad.libs.inertia as inertia
from zencad.libs.screw import screw
import zencad.libs.kinematic as kinematic

import numpy

numpy.set_printoptions(precision=2, linewidth=160)

L = 100
arot = kinematic.rotator(name="AROT",ax=(1,0,0))
brot = kinematic.rotator(name="BROT",ax=(1,0,0))
a = zencad.assemble.unit(name="A")
b = zencad.assemble.unit(name="B")
ma = zencad.assemble.unit(name="MA")
mb = zencad.assemble.unit(name="MB", shape=sphere(10))

a.add_shape(cylinder(r=5, h=L))
b.add_shape(cylinder(r=5, h=L))

base = zencad.assemble.unit()

base.link(arot)
arot.link(a)
a.link(brot)
brot.link(b)

brot.relocate(up(L))

a.link(ma)
b.link(mb)

ma.relocate(up(L))
mb.relocate(up(L))

arot.dempher_koeff = 0
brot.dempher_koeff = 0

#treedy.attach_inertia(a, mass=1, Ix=0, Iy=0, Iz=0, pose=up(L/2))
#treedy.attach_inertia(b, mass=1, Ix=0, Iy=0, Iz=0, pose=up(L/2))
inertia.attach_inertia(ma, mass=0.00001, Ix=0.00001, Iy=0.00001, Iz=0.00001)
inertia.attach_inertia(mb, mass=1, Ix=1, Iy=1, Iz=1)

#forces.gravity(unit=a,vec=(0,0,-9081))
#forces.gravity(unit=b,vec=(0,0,-9081))
#forces.gravity(unit=ma,vec=(0,0,-9081))
#forces.gravity(unit=mb,vec=(0,0,-9081))

#dph1 = forces.dempher(unit=arot, koeff=50000)
#dph2 = forces.dempher(unit=brot, koeff=50000)

base.relocate(rotateZ(deg(90)))
arot.set_coord(deg(90))

brot.set_speed(2)
t = treedy.tree_dynamic_solver(base)

#t.onestep(0.0001)
numpy.set_printoptions(suppress=True)
print(t.reaction_solver.inertia_forces())
print(t.reaction_solver.constrait_matrix()[0])
print(t.reaction_solver.mass_matrix())
exit(0)

#t.print_reaction_lagrange_multipliers()
#t.print_local_inertial_objects()

#print(brot.frame_inertia)

#t.onestep(0.0001)

#print(t.reaction_solver.solve())

#print(t.reaction_solver.constraits)
#print(t.reaction_solver.rigid_bodies)
#
#for r in t.reaction_solver.rigid_bodies:
#	print(r.constrait_connections)
#
#exit(0)

#t.onestep(delta=0.01)

#print(arot.rigid_body.constrait_connections[0].constrait.constrait_screws())
#print(arot.rigid_body.constrait_connections[0].constrait_screws())
#print(arot.rigid_body.constrait_connections[0].constraits_screws_in_body_frame())

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
		maxdelta = 0.02
		curtime = time.time()
		delta = curtime - lasttime
		lasttime = curtime


		if delta > maxdelta:
			delta = maxdelta
		DELTA = delta

		t.onestep(delta=delta)
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