#!/usr/bin/env python3

import pyservoce
import zencad
import zencad.libs.matrix_solver
import zencad.libs.constraits as constraits
from zencad.libs.rigid_body import rigid_body
import zencad.libs.kinematic as kinematic
from zencad.libs.inertia import inertia
from zencad.libs.screw import screw

a = rigid_body(inertia=inertia(), global_pose=zencad.transform.right(10))
b = rigid_body(inertia=inertia(cm = pyservoce.point3(0,0,0)), global_pose=zencad.transform.right(20))

c1 = constraits.rotator_constrait(ax=(0,0,1))
c1.attach_positive_connection(body=a, radius=pyservoce.vector3(-10,0,0))

c = constraits.rotator_constrait(ax=(0,0,1))
c.attach_positive_connection(body=a, radius=pyservoce.vector3(10,0,0))
c.attach_negative_connection(body=b, radius=pyservoce.vector3(-10,0,0))

b.set_global_speed(screw(lin=(0,0,0), ang=(0,0,2)))


solver = zencad.libs.matrix_solver.reaction_solver(
	rigid_bodies=[a,b],
	constraits=[c,c1]
)

solver.update_globals()

print(a.global_inertia)
print(b.global_inertia)
print(a.inertia_force())
print(b.inertia_force())

print("G")
G, h = solver.constrait_matrix()
print(G)
print("h")
print(h)

print("K")
print(solver.inertia_forces())

print("M")
print(solver.mass_matrix())

print("Reaction:")
print(solver.solve())