import zencad.transform
from zencad.libs.screw import screw

class physunit(zencad.assemble.unit):
	def __init__(self, pose, )


#def attach_force_model(unit):
#	unit.force_sources = []
#	self.output_force = screw()
#	for ch in self.childs:
#		unit.force_sources.append(ch)


class dof_agent:
	def __init__(self, unit, parent=None):
		self.parent = parent
		self.unit = unit

	def dof_divide(self):
		raise NotImplementedError

class free_dof_agent(dof_agent):
	def dof_divide(self, force):
		self.dalamber = force
		self.reaction = screw()

class world:
	def __init__(self):
		self.free_units = []

	def add_free_unit(self, unit):
		self.free_units.append(unit)

	def integrate(self):
		pass

	def onestep(self, delta):
		print("onestep")
