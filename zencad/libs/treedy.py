#!/usr/bin/env pyython3

import pyservoce
import zencad.libs.kinematic
import zencad.libs.inertia

class tree_dynamic_solver:
	def __init__(self, baseunit):
		self.baseunit = baseunit
		
		self.inertial_objects = []
		self.kinematic_frames = []
		
		self.find_all_inertial_objects(baseunit, self.inertial_objects)
		self.find_all_kinematic_frames(baseunit, self.kinematic_frames)

		for i in range(len(self.inertial_objects)): self.inertial_objects[i].dynno = i
		for i in range(len(self.kinematic_frames)): self.kinematic_frames[i].dynno = i

	def find_post_inertia_objects(self):
		for f in self.kinematic_frames:
			f.post_inertia_objects = []
			 
			def iteration(unit):
				if hasattr(unit, "inertial_object"):
					f.post_inertia_objects.append(unit.inertial_object)
				for u in unit.childs:
					iteration(u)

			iteration(f)

	def find_all_inertial_objects(self, unit, retarr):
		if hasattr(unit, "inertial_object"):
			retarr.append(unit.inertial_object)

		for u in unit.childs:
			self.find_all_inertial_objects(u, retarr)

	def find_all_kinematic_frames(self, unit):
		if isinstance(unit, zencad.libs.kinematic.kinematic_frame):
			retarr.append(unit)

		for u in unit.childs:
			self.find_all_kinematic_frames(u, retarr)

	def mass_matrix(self):
		pass