import zencad.libs.physics
from zencad.libs.screw import screw
import pyservoce

class gravity(zencad.libs.physics.force_source):
	def __init__(self, unit, vec=pyservoce.vector3(0,0,-9.81)):
		self.unit = unit
		self.g = screw(lin=vec, ang=(0,0,0))

	def evaluate(self):
		pose = self.unit.global_location
		rot = pose.rotation().inverse()
		self.force = self.g.rotate_by_quat(rot) * self.unit.inertia.mass
		self.center = pyservoce.vector3(*self.unit.inertia.cm)