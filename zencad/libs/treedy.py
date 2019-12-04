#!/usr/bin/env pyython3

import pyservoce
import zencad.libs.kinematic
import zencad.libs.inertia
from zencad.libs.screw import screw

from zencad.libs.kinematic import kinematic_frame

class tree_dynamic_solver:
	def __init__(self, baseunit):
		self.baseunit = baseunit
		
		self.inertial_objects = []
		self.kinematic_frames = []
		self.force_sources = []
		self.constrait_conditions = []
		self.units = []

		self.find_all_inertial_objects(baseunit, self.inertial_objects)
		self.find_all_kinematic_frames(baseunit, self.kinematic_frames)
		self.find_all_force_sources(baseunit, self.force_sources)
		self.find_all_constrait_conditions(baseunit, self.constrait_conditions)
		self.find_all_units(baseunit, self.units)

		for i in range(len(self.inertial_objects)): self.inertial_objects[i].dynno = i
		for i in range(len(self.kinematic_frames)): self.kinematic_frames[i].dynno = i
		for i in range(len(self.force_sources)): self.force_sources[i].dynno = i
		for i in range(len(self.constrait_conditions)): self.constrait_conditions[i].dynno = i

	#	self.find_post_inertia_objects()
		self.find_pre_kinematic_frames()

	def print_state(self):
		print("kinematic_frames", self.kinematic_frames)
		print("inertial_objects", self.inertial_objects)
		print("force_sources", self.force_sources)
		print("constrait_conditions", self.constrait_conditions)
		print("units", self.units)

	def print_pre_kinematic_frames(self):
		print("pre_kinematic_frames for iners:")
		for iner in self.inertial_objects:
			print("{}: {}".format(iner.unit.name, iner.pre_kinematic_frames))

	def find_pre_kinematic_frames(self):
		for iner in self.inertial_objects:
			u = iner.unit
			iner.pre_kinematic_frames = []
			while u is not None:
				if isinstance(u, kinematic_frame):
					iner.pre_kinematic_frames.append(u)
				u = u.parent

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

	def find_all_kinematic_frames(self, unit, retarr):
		if isinstance(unit, zencad.libs.kinematic.kinematic_frame):
			retarr.append(unit)

		for u in unit.childs:
			self.find_all_kinematic_frames(u, retarr)

	def find_all_force_sources(self, unit, retarr):
		if hasattr(unit, "force_sources"):
			self.force_sources = self.force_sources + unit.force_sources

		for u in unit.childs:
			self.find_all_force_sources(u, retarr)

	def find_all_constrait_conditions(self, unit, retarr):
		if hasattr(unit, "force_sources"):
			self.constrait_conditions = \
				self.constrait_conditions + unit.constrait_conditions

		for u in unit.childs:
			self.find_all_constrait_conditions(u, retarr)

	def find_all_units(self, unit, retarr):
		self.units.append(unit)

		for u in unit.childs:
			self.find_all_units(u, retarr)

	def mass_matrix(self):
		pass

	def calculate_impulses(self):
		for iner in self.inertial_objects:
			accum = screw()
			for p in iner.pre_kinematic_frames:
				accum += p.global_spdscr
			iner.global_update_impulse_with_speed(accum)	

	def print_impulses(self):
		for i in self.inertial_objects:
			print("{}: {}".format(i.unit, i.global_impulse))

def attach_inertia(unit, 
		pose=zencad.transform.nulltrans(), 
		mass=1, 
		Ix=1, Iy=1, Iz=1, Ixy=0, Ixz=0, Iyz=0):
	unit.inertial_object = zencad.libs.inertia.inertial_object( 
		unit=unit, pose=pose, mass=1, Ix=1, Iy=1, Iz=1, Ixy=0, Ixz=0, Iyz=0)