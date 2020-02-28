#!/usr/bin/env python3
"""
ZenCad API example: Color object animation.
last update: 13.10.2019
"""

from zencad import *
import random

import zencad.platonic

s = zencad.platonic.icosahedron(10)
#s = zencad.platonic.dodecahedron(10)
#s = zencad.platonic.icosahedron(10).fillet(2)

controller = disp(s)

transparent = 0.05

clr = color(0.5,0.5,0.5,transparent)
def animate(wdg):
	global clr

	def change(old):
		new = old + random.uniform(-0.02, 0.02)
		if new > 1 or new < 0:
			return old
		return new

	clr = color(change(clr.r), change(clr.g), change(clr.b), transparent)
	controller.set_color(clr)

show(animate=animate)
