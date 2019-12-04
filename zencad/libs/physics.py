import zencad.transform
from zencad.libs.screw import screw
import zencad.libs.inertia
import pyservoce

class force_source:
	def __init__(self, finit=screw()):
		self.force = finit

	def evaluate(self):
		raise NotImplementedError

	def attach(self, unit):
		if not hasattr(unit, "force_sources"):
			unit.force_sources = []
		unit.force_sources.append(self)

class physunit(zencad.assemble.unit):
	def __init__(self, 
			parent=None, 
			location=zencad.transform.nulltrans(), 
			inertia=zencad.libs.inertia.inertia()):
		super().__init__(parent, location)
		self.inertia = inertia
		self.force_srcs = []		
		self.cm_absframe_speed = screw()
		self.cm_absframe_accel = screw()

#	def restore_kinchain_spdacc(self):
#		w = vector3()
#		for kinframe in self.kinematic_chains.kinframes:
#			w += kinframe.absframe_angular_speed()
#		self.korriolis = 
#			self.mass * self.cm_absframe_speed.lin.cross(w) * 2
#		self.hyroscopic = self.cm_absframe.kinmoment.cross(w)

	def add_force_source(self, fsource):
		self.force_srcs.append(fsource)

	def reduce_forces(self):
		fscrew = screw()

		for u in self.childs:
			u.reduce_forces()
			mov = - u.location.translation()
			rot = u.location.rotation().inverse()
			reaction_of_unit = u.reaction.rotate_by_quat(rot).carry(-mov)
			fscrew += reaction_of_unit

		for fs in self.force_srcs:
			fs.evaluate()
			fscrew += fs.force.carry(-fs.center)

		self.reaction = fscrew

	def evaluate_complex_inertia(self):
		for u in self.childs:
			u.evaluate_complex_inertia()

		lst = [ u.complex_inertia for u in self.childs ]
		lst.append(self.inertia)
		self.complex_inertia = zencad.libs.inertia.complex_inertia(lst)

	def evaluate_accelerations(self):
		print(self.complex_inertia)
		dalamber = self.dalamber.carry(
			pyservoce.vector3(*self.complex_inertia.cm))


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
