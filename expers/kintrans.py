#!/usr/bin/env python3

from zencad import *
import zencad.assemble as assemble
from zencad.mbody import kinematic 
from zencad.mbody.kintranslator import kintranslator
from zencad.mbody.solver import matrix_solver
import zencad.libs.inertia as inertia

class link(assemble.unit):
	def __init__(self,r=3,h=20):
		super().__init__()
		self.add_shape(cylinder(r,h))
		inertia.attach_inertia(self, inertia.inertia(mass=1, matrix=pyservoce.matrix33(1,1,1), radius=(0,0,h/2)))
		self.out = kinematic.rotator(ax=(1,0,0), pose=up(h))

brot = kinematic.rotator(ax=(1,0,0))
a = link()
b = link()
c = link()

brot.link(a)
a.out.link(b)
b.out.link(c)

translator = kintranslator(brot, mode=kintranslator.CONSTRAIT_MODE)
rigids, constraits = translator.build()

matrix_solver(bodies=rigids, constraits=constraits)
