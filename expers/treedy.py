#!/usr/bin/env python3

import zencad
from zencad import *
import time
import zencad.libs.treedy as treedy
import zencad.libs.forces as forces
from zencad.libs.screw import screw
import zencad.libs.kinematic as kinematic

body = zencad.assemble.unit(name="N", shape=sphere(20))
a = zencad.assemble.unit(name="A", shape=(cylinder(h=40, r=5)+sphere(10).up(40)).rotateX(deg(90)))
b = zencad.assemble.unit(name="B", shape=(cylinder(h=40, r=5)+sphere(10).up(40)).rotateX(deg(90)))

ra = kinematic.rotator(name="RA",ax=(1,0,0), location=translate(0,-20,0), parent=body)
rb = kinematic.rotator(name="RB",ax=(1,0,0), location=translate(0,20,0)*rotateZ(deg(180)), parent=body)

ra.link(a)
rb.link(b)


treedy.attach_inertia(a, mass=1, Ix=1, Iy=1, Iz=1, pose=forw(40))
treedy.attach_inertia(b, mass=1, Ix=1, Iy=1, Iz=1, pose=forw(40))

forces.gravity(unit=a,vec=(0,0,-0.2)).attach(a)
forces.gravity(unit=b,vec=(0,0,-0.2)).attach(b)

t = treedy.tree_dynamic_solver(body)

#s.set_speed_screw(screw(lin=(0.1,0.1,0.1),ang=(0,0,0)))

#t.calculate_impulses()

t.print_state()
t.print_pre_kinematic_frames()
t.print_post_inertial_objects()
t.print_post_force_sources()
t.print_impulses()

starttime = time.time()
lasttime = starttime
def animate(self):
	global lasttime
	curtime = time.time()
	delta = curtime - lasttime
	lasttime = curtime

	t.onestep(delta=delta)
	print(ra.global_force_reduction)
	print(ra.acceleration)
	print(ra.speed)

disp(body)
show(animate=animate)