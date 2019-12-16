import zencad.libs.screw
from zencad.libs.screw import screw
import zencad.trans

class force_unit:
	def __init__(self, unit, location=nulltrans()):
		self.unit = unit
		self.childs = []
		self.location = location

	def local_force(self):
		return screw(lin=(0,0,0), ang=(0,0,0))

	def force(self):

		return self.local_force()