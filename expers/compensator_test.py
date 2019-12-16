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
numpy.set_printoptions(precision=3, linewidth=160)

L=20

#body = zencad.box(30, center= True)

l00 = zencad.cylinder(r=5, h=L, center=True)
l01 = zencad.cylinder(r=5, h=L, center=True)
l10 = zencad.cylinder(r=5, h=L, center=True)
l11 = zencad.cylinder(r=5, h=L, center=True)
l100 = zencad.cylinder(r=5, h=L, center=True)
l101 = zencad.cylinder(r=5, h=L, center=True)
l110 = zencad.cylinder(r=5, h=L, center=True)
l111 = zencad.cylinder(r=5, h=L, center=True)

#rbody = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(0,0,0))
rl00 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(40,10,-40))
rl01 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(40,20,-80))
rl10 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(-40,-15,-40))
rl11 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(-40,13,-80))
rl100 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(40,-50,40))
rl101 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(40,6,80))
rl110 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(-40,80,40))
rl111 = rigid_body(inertia=inertia(radius = pyservoce.vector3(0,0,0)), pose=zencad.translate(-40,30,80))

#rbody.add_view(disp(body))
rl00.add_view(disp(l00))
rl10.add_view(disp(l10))
rl01.add_view(disp(l01))
rl11.add_view(disp(l11))
rl100.add_view(disp(l100))
rl110.add_view(disp(l110))
rl101.add_view(disp(l101))
rl111.add_view(disp(l111))

T_comp=10

c00 = constraits.spherical_rotator(T_comp=T_comp)
c01 = constraits.spherical_rotator(T_comp=T_comp)
c10 = constraits.spherical_rotator(T_comp=T_comp)
c11 = constraits.spherical_rotator(T_comp=T_comp)
c100 = constraits.spherical_rotator(T_comp=T_comp)
c101 = constraits.spherical_rotator(T_comp=T_comp)
c110 = constraits.spherical_rotator(T_comp=T_comp)
c111 = constraits.spherical_rotator(T_comp=T_comp)

#c00.attach(rbody, translate(15,0,-15), rl00, translate(0,0,L/2))
#c01.attach(rbody, translate(-15,0,-15), rl10, translate(0,0,L/2))
#c10.attach(rl10, translate(0,0,-L/2), rl11, translate(0,0,L/2))
#c11.attach(rl00, translate(0,0,-L/2), rl01, translate(0,0,L/2))
#
#c100.attach(rbody, translate(15,0,15), rl100, translate(0,0,-L/2))
#c101.attach(rbody, translate(-15,0,15), rl110, translate(0,0,-L/2))
#c110.attach(rl110, translate(0,0,L/2), rl111, translate(0,0,-L/2))
#c111.attach(rl100, translate(0,0,L/2), rl101, translate(0,0,-L/2))

c00.attach(rl00, translate(0,0,L/2), rl01, translate(0,0,-L/2))
c01.attach(rl01, translate(0,0,L/2), rl10, translate(0,0,-L/2))
c10.attach(rl10, translate(0,0,L/2), rl11, translate(0,0,-L/2))
c11.attach(rl11, translate(0,0,L/2), rl100, translate(0,0,-L/2))

c100.attach(rl100, translate(0,0,L/2), rl101, translate(0,0,-L/2))
c101.attach(rl101, translate(0,0,L/2), rl110, translate(0,0,-L/2))
c110.attach(rl110, translate(0,0,L/2), rl111, translate(0,0,-L/2))
#c111.attach(rl111, translate(0,0,0), rl00, translate(0,0,0))


solver = zencad.mbody.solver.matrix_solver(
	rigid_bodies=[rl00,rl10,rl11,rl01, rl100,rl110,rl111,rl101], 
	constraits=[c00,c01,c10,c11,c100,c101,c110])
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

	maxdelta = 10
	curtime = time.time()
	delta = curtime - lasttime
	lasttime = curtime

	if delta > maxdelta:
		delta = maxdelta
	DELTA = delta

	solver.solve()
	solver.apply(DELTA)

show(animate=animate, animate_step = 0.00000001)