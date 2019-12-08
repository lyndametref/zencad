from zencad.libs.screw import screw
from zencad.libs.kinematic import *
from zencad.libs.dynsolver import dynamic_solver

class lagrange_solver(dynamic_solver):
	no = 0

	@classmethod
	def get_multiplier(cls):
		cls.no += 1
		return cls.no

	def __init__(self, baseunit):
		super().__init__(baseunit)
		self.set_lagrange_multipliers(baseunit)

	def find_all_inertial_objects(self, unit, retarr):
		if hasattr(unit, "inertial_object"):
			retarr.append(unit.inertial_object)

		for u in unit.childs:
			self.find_all_inertial_objects(u, retarr)

	def find_all_kinematic_frames(self, unit, retarr):
		if isinstance(unit, zencad.libs.kinematic.kinematic_frame):
			retarr.append(unit)

		for u in unit.childs:
			self.find_all_kinematic_frames(u, retarr)

	def set_lagrange_multipliers(self, unit):
		if isinstance(unit, kinematic_frame):
			reaction_quantity = unit.reaction_quantity
			lagrange_numbers = [ self.get_multiplier() for i in range(reaction_quantity) ]
			unit.reaction_lagrange_numbers = lagrange_numbers

		for u in unit.childs:
			self.set_lagrange_multipliers(u)		

	def print_reaction_lagrange_multipliers(self):
		for kinframe in self.kinematic_frames:
			print(kinframe.name, kinframe.reaction_lagrange_numbers)


	def onestep(self, delta):
		pass