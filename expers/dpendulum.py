#!/usr/bin/env python3

import zencad
from zencad import *
import time
import zencad.libs.treedy as treedy
import zencad.libs.forces as forces
from zencad.libs.screw import screw
import zencad.libs.kinematic as kinematic

L = 100
arot = kinematic.rotator(name="AROT",ax=(1,0,0))
brot = kinematic.rotator(name="BROT",ax=(1,0,0))
a = zencad.assemble.unit(name="A")
b = zencad.assemble.unit(name="B")
ma = zencad.assemble.unit(name="MA")
mb = zencad.assemble.unit(name="MB")

a.add_shape(cylinder(r=5, h=L))
b.add_shape(cylinder(r=5, h=L))

arot.link(a)
a.link(brot)
brot.link(b)

brot.relocate(up(L))

arot.set_coord(deg(90))

a.link(ma)
b.link(mb)

ma.relocate(up(L))
mb.relocate(up(L))

arot.dempher_koeff = 0
brot.dempher_koeff = 0

treedy.attach_inertia(ma, mass=1, Ix=0, Iy=0, Iz=0)
treedy.attach_inertia(mb, mass=1, Ix=0, Iy=0, Iz=0)

forces.gravity(unit=ma,vec=(0,0,-5081)).attach(ma)
forces.gravity(unit=mb,vec=(0,0,-5081)).attach(mb)
dph1 = forces.dempher(unit=arot, koeff=50000)
dph2 = forces.dempher(unit=brot, koeff=50000)

arot.set_speed(1)
t = treedy.tree_dynamic_solver(arot)
t.set_linear_scale(1)

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
		maxdelta = 0.1
		curtime = time.time()
		delta = curtime - lasttime
		lasttime = curtime


		if delta > maxdelta:
			delta = maxdelta
		DELTA = delta

		t.onestep(delta=delta)

def animate(self):
	arot.location_update(deep=True, view=True)

import threading
evalthr = threading.Thread(target=evaluate)
evalthr.start()

def close_handle():
	global cancel
	cancel = True

disp(arot)
show(animate=animate, animate_step = 0.00001, close_handle=close_handle)