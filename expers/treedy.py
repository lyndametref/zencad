#!/usr/bin/env python3

import zencad
import zencad.libs.treedy as treedy
from zencad.libs.screw import screw
import zencad.libs.kinematic as kinematic

a = zencad.assemble.unit(name="A")
b = zencad.assemble.unit(name="B")
r = kinematic.rotator(name="R",ax=(0,0,1))
s = kinematic.space(name="Space")

treedy.attach_inertia(a, mass=1, Ix=1, Iy=1, Iz=1)

s.link(a)
a.link(r)
r.link(b)

t = treedy.tree_dynamic_solver(s)

s.set_speed_screw(screw(lin=(1,1,1),ang=(0,0,0)))

t.calculate_impulses()

t.print_state()
t.print_pre_kinematic_frames()
t.print_impulses()