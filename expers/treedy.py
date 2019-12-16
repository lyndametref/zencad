#!/usr/bin/env python3

import cProfile
import numpy
import zencad
from zencad import *
import time
import zencad.libs.treedy as treedy
import zencad.libs.forces as forces
from zencad.libs.screw import screw
import zencad.mbody.kinematic as kinematic
import zencad.libs.inertia as inertia

L = 100
rot = kinematic.rotator(name="RBODY",ax=(0,0,1))
body = zencad.assemble.unit(name="N", shape=sphere(20))
a = zencad.assemble.unit(name="A")
b = zencad.assemble.unit(name="B")
aa = zencad.assemble.unit(name="AA")
bb = zencad.assemble.unit(name="BB")
aaa = zencad.assemble.unit(name="AAA")
bbb = zencad.assemble.unit(name="BBB")
aaaa = zencad.assemble.unit(name="AAAA")
bbbb = zencad.assemble.unit(name="BBBB")
aaaaa = zencad.assemble.unit(name="AAAAA")
bbbbb = zencad.assemble.unit(name="BBBBB")

a.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)), color=zencad.color.blue)
b.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)))
aa.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)), color=zencad.color.blue)
bb.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)))
aaa.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)), color=zencad.color.blue)
bbb.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)))
aaaa.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)), color=zencad.color.blue)
bbbb.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)))
aaaaa.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)), color=zencad.color.blue)
bbbbb.add_shape((cylinder(h=L, r=5)+sphere(10).up(L)))
inertia.attach_inertia(rot.output, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(body, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(a, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(b, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(aa, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(bb, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(aaa, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(bbb, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(aaaa, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(bbbb, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(aaaaa, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(bbbbb, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))

rra = kinematic.rotator(name="RRA",ax=(0,0,1), location=translate(0,-20,0), parent=body)
rrb = kinematic.rotator(name="RRB",ax=(0,0,1), location=translate(0,20,0)*rotateZ(deg(180)), parent=body)
ra = kinematic.rotator(name="RA",ax=(1,0,0))
rb = kinematic.rotator(name="RB",ax=(1,0,0))
raa = kinematic.rotator(name="RAA",ax=(0,1,0))
rbb = kinematic.rotator(name="RBB",ax=(0,1,0))
raaa = kinematic.rotator(name="RAAA",ax=(0,1,0))
rbbb = kinematic.rotator(name="RBBB",ax=(0,1,0))
raaaa = kinematic.rotator(name="RAAAA",ax=(0,1,0))
rbbbb = kinematic.rotator(name="RBBBB",ax=(0,1,0))
raaaaa = kinematic.rotator(name="RAAAAA",ax=(0,1,0))
rbbbbb = kinematic.rotator(name="RBBBBB",ax=(0,1,0))

inertia.attach_inertia(rra.output, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))
inertia.attach_inertia(rrb.output, mass=0.1, Ix=0.1, Iy=0.1, Iz=0.1, radius=pyservoce.vector3(0,0,L))

raa.relocate(up(L))
rbb.relocate(up(L))
raaa.relocate(up(L))
rbbb.relocate(up(L))
raaaa.relocate(up(L))
rbbbb.relocate(up(L))
raaaaa.relocate(up(L))
rbbbbb.relocate(up(L))


rra.link_directly(ra)
rrb.link_directly(rb)
ra.link_directly(a)
rb.link_directly(b)
a.link(raa)
b.link(rbb)
raa.link_directly(aa)
rbb.link_directly(bb)
aa.link(raaa)
bb.link(rbbb)
raaa.link_directly(aaa)
rbbb.link_directly(bbb)
aaa.link(raaaa)
bbb.link(rbbbb)
raaaa.link_directly(aaaa)
rbbbb.link_directly(bbbb)
aaaa.link(raaaaa)
bbbb.link(rbbbbb)
raaaaa.link_directly(aaaaa)
rbbbbb.link_directly(bbbbb)
rot.link_directly(body)

#rra.set_coord(deg(20))
#rra.set_coord(deg(10))
#ra.set_coord(deg(90))
#rb.set_coord(deg(90))

rot.location_update()

#forces.gravity(unit=a,vec=(0,0,-5081))
#forces.gravity(unit=b,vec=(0,0,-5081))
#forces.gravity(unit=aa,vec=(0,0,-5081))
#forces.gravity(unit=bb,vec=(0,0,-5081))
#forces.gravity(unit=aaa,vec=(0,0,-5081))
#forces.gravity(unit=bbb,vec=(0,0,-5081))
#forces.gravity(unit=aaaa,vec=(0,0,-5081))
#forces.gravity(unit=bbbb,vec=(0,0,-5081))
#forces.gravity(unit=aaaaa,vec=(0,0,-5081))
#forces.gravity(unit=bbbbb,vec=(0,0,-5081))

KD = 0
ra.dempher_koeff = KD
rb.dempher_koeff = KD
rra.dempher_koeff = KD
rrb.dempher_koeff = KD
raa.dempher_koeff = KD
rbb.dempher_koeff = KD
raaa.dempher_koeff = KD
rbbb.dempher_koeff = KD
raaaa.dempher_koeff = KD
rbbbb.dempher_koeff = KD
raaaaa.dempher_koeff = KD
rbbbbb.dempher_koeff = KD

#KD = 0
#mot = forces.motor(unit=rot)
#dph_rra = forces.dempher(unit=rra, koeff=KD)
#dph_ra = forces.dempher(unit=ra, koeff=KD)
#dph_raa = forces.dempher(unit=raa, koeff=KD)
#dph_raaa = forces.dempher(unit=raaa, koeff=KD)
#dph_raaaa = forces.dempher(unit=raaaa, koeff=KD)
#dph_raaaaa = forces.dempher(unit=raaaaa, koeff=KD)
#dph_rrb = forces.dempher(unit=rrb, koeff=KD)
#dph_rb = forces.dempher(unit=rb, koeff=KD)
#dph_rbb = forces.dempher(unit=rbb, koeff=KD)
#dph_rbbb = forces.dempher(unit=rbbb, koeff=KD)
#dph_rbbbb = forces.dempher(unit=rbbbb, koeff=KD)
#dph_rbbbbb = forces.dempher(unit=rbbbbb, koeff=KD)

#rot.set_coord(deg(10))

t = treedy.tree_dynamic_solver(rot)

numpy.set_printoptions(threshold=sys.maxsize, linewidth=10000)
print(t.reaction_solver.mass_matrix())

#s.set_speed_screw(screw(lin=(0.1,0.1,0.1),ang=(0,0,0)))

#t.calculate_impulses()


cancel = False
DELTA = 0

cp = cProfile.Profile()

def evaluate():
	global DELTA
	starttime = time.time()
	lasttime = starttime
	modeltime = 0
	time.sleep(2)

	#cp.enable()
	while True:
		if cancel:
			return
		maxdelta = 0.01
		curtime = time.time()
		delta = curtime - lasttime
		lasttime = curtime
		#print("DELTA", delta)
		#rot.set_speed(5)

		if delta > maxdelta:
			delta = maxdelta
		DELTA = delta

		modeltime +=delta
		spd = math.cos((modeltime)*2) + 1.5
		rot.set_speed(4 * spd)
		#mot.set_moment(100)

		print("HERE", rot.global_force_reduction)


		t.onestep(delta=delta)
#		cp.run("t.onestep(DELTA)")
		#print(ra.inertia_koefficient)
		#exit(0)

		print(ra.global_force_reduction)
		#print(ra.acceleration)
		print(ra.inertia_koefficient)

def animate(self):
	rot.location_update(deep=True, view=True)

import threading
evalthr = threading.Thread(target=evaluate)
evalthr.start()

def close_handle():
	global cancel
	cancel = True
	#cp.disable()
	#cp.print_stats(sort=1)

disp(rot)
show(animate=animate, animate_step = 0.00001, close_handle=close_handle)