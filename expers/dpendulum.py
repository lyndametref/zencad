#!/usr/bin/env python3

import zencad
from zencad import *
import numpy.linalg
import time
import zencad.libs.treedy as treedy
import zencad.libs.forces as forces
import zencad.libs.inertia as inertia
from zencad.libs.screw import screw
import zencad.mbody.kinematic as kinematic

import numpy

numpy.set_printoptions(precision=5, linewidth=200)

L = 100
arot = kinematic.rotator(name="AROT",ax=(1,0,0))
brot = kinematic.rotator(name="BROT",ax=(1,0,0))
crot = kinematic.rotator(name="CROT",ax=(1,0,0))
a = zencad.assemble.unit(name="A")
b = zencad.assemble.unit(name="B")
c = zencad.assemble.unit(name="C")
ma = zencad.assemble.unit(name="MA", shape=sphere(10))
mb = zencad.assemble.unit(name="MB", shape=sphere(10))
mc = zencad.assemble.unit(name="MC", shape=sphere(10))

a.add_shape(cylinder(r=5, h=L))
b.add_shape(cylinder(r=5, h=L))
c.add_shape(cylinder(r=5, h=L))

base = zencad.assemble.unit()

base.link(arot)
arot.link(a)
a.link(brot)
brot.link(b)
#b.link(crot)
crot.link(c)

brot.relocate(up(L))
crot.relocate(up(L))

a.link(ma)
b.link(mb)
c.link(mc)

ma.relocate(up(L))
mb.relocate(up(L))
mc.relocate(up(L))

arot.dempher_koeff = 0
brot.dempher_koeff = 0

#treedy.attach_inertia(a, mass=1, Ix=0, Iy=0, Iz=0, pose=up(L/2))
#treedy.attach_inertia(b, mass=1, Ix=0, Iy=0, Iz=0, pose=up(L/2))
inertia.attach_inertia(ma, mass=1, Ix=1, Iy=1, Iz=1)
inertia.attach_inertia(mb, mass=1, Ix=1, Iy=1, Iz=1)
inertia.attach_inertia(mc, mass=1, Ix=1, Iy=1, Iz=1)

#forces.gravity(unit=a,vec=(0,0,-9081))
#forces.gravity(unit=b,vec=(0,0,-9081))
#forces.gravity(unit=ma,vec=(0,0,-9081))
#forces.gravity(unit=mb,vec=(0,0,-9081))

#dph1 = forces.dempher(unit=arot, koeff=50000)
#dph2 = forces.dempher(unit=brot, koeff=50000)

base.relocate(rotateZ(deg(0)))
arot.set_coord(deg(0))

brot.set_speed(2)
t = treedy.tree_dynamic_solver(base)

numpy.set_printoptions(suppress=True)
print(arot.rigid_body.global_inertia.to_mass_matrix())
print(t.reaction_solver.mass_matrix())

t.onestep(0.0001)

print("K")
print(t.reaction_solver.inertia_forces())

print("G")
print(t.reaction_solver.constrait_matrix()[0])

print("M")
print(t.reaction_solver.mass_matrix())

"solve"
print(t.reaction_solver.solve())

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
		maxdelta = 0.001
		curtime = time.time()
		delta = curtime - lasttime
		lasttime = curtime


		if delta > maxdelta:
			delta = maxdelta
		DELTA = delta

		try:
			t.onestep(delta=delta)
		except:
			print(arot.rigid_body.global_inertia)
			print(t.reaction_solver.mass_matrix())
			exit(0)
		time.sleep(0.01)

		#print(t.reaction_solver.mass_matrix())
		#print(t.reaction_solver.constrait_matrix()[0])
		#print(t.reaction_solver.reactions)

		print(t.reaction_solver.accelerations)

		#print(t.reaction_solver.reactions)
		
		#print(t.reaction_solver.constraits[0].connections[0].get_reaction_force_global())
		#print(t.reaction_solver.constraits[1].connections[0].get_reaction_force_global())
		#print(t.reaction_solver.constraits[1].connections[1].get_reaction_force_global())

		#print(crot.output.global_frame_speed_reference)
		#print(crot.output.global_frame_acceleration_reference)
		#print(crot.rigid_body.global_inertia.radius)
		#print(crot.rigid_body.global_inertia)

		#print(mb, mb.global_frame_acceleration_reference)
		#print(ma, ma.global_frame_acceleration_reference)


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