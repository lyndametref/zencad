#!/usr/bin/env python3

from zencad import *
import zencad.libs.physics
import zencad.libs.kinematic
import zencad.libs.forces
from zencad.libs.inertia import inertia

class body(zencad.libs.physics.physunit):
	def __init__(self):
		super().__init__(inertia=inertia(mass=2))
		self.add_shape(sphere(10))
		self.add_force_source(zencad.libs.forces.gravity(unit=self))

b_dof = zencad.libs.kinematic.free()
b = body()

b_dof.link(b)

root = b_dof
root.reduce_forces()
root.evaluate_complex_inertia()
root.evaluate_accelerations()

print(b_dof.prereaction)
print(b_dof.reaction)
print(b_dof.dalamber)