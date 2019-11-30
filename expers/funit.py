#!/usr/bin/env python3

from zencad import *
import zencad.libs.physics
from zencad.libs.fmodel import \
	constant_gravity_funit, \
	evaluate_unit_force

b = zencad.assemble.unit()
b.add_shape(box(20, center=True), color=color(1,0,0,0.5))
b.relocate(translate(20,20,20) * rotateX(deg(45)))
#b.location_update()

b_gravity = constant_gravity_funit(unit=b)
#b_gravity.draw_vector(scale=20)

print( evaluate_unit_force(b) )

disp(b)
show()