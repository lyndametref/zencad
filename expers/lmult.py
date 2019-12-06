#!/usr/bin/env python3

import zencad.libs.lagrange
import pyservoce
from zencad.libs.lagrange import make_new_lagrange_polynom
from zencad.libs.screw import screw
from zencad.libs.inertia import inertia

i = inertia(mass=1, matrix=pyservoce.matrix33(1,2,1))

l = make_new_lagrange_polynom([
	screw(ang=(0,1,0)),
	screw(ang=(1,0,0)),
])

ll = l.foreach(lambda acc: i.acceleration_to_force(acc))

print(ll)