#!/usr/bin/env python3

from zencad import *
import zencad.mbody.kinematic

from zencad.mbody.kintranslator import kintranslator

class link(zencad.assemble.unit):
	def __init__(self, r=3, h=20):
		super().__init__()
		self.add_shape(cylinder(r, h))
		self.rot = zencad.mbody.kinematic.rotator(parent=self, ax=(0,1,0), pose=moveZ(h))


baserot = zencad.mbody.kinematic.rotator(ax=(0,1,0))
u = link()

baserot.link(u)
baserot.set_coord(deg(90))

kintrans = kintranslator(baserot)
rigids, constraits = kintrans.build(mode=kintranslator.CONSTRAIT_MODE)

def animate(self):
	baserot.update_pose()

disp(baserot)
show(animate=animate, animate_step = 0.001)