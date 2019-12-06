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

		self.find_post_force_sources()
		self.find_post_inertial_objects()
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

	def print_post_inertial_objects(self):
		print("post_inertial_objects for kinframes:")
		for kinframe in self.kinematic_frames:
			print("{}: {}".format(kinframe, kinframe.post_inertial_objects))

	def print_post_force_sources(self):
		print("post_force_sources for kinframes:")
		for kinframe in self.kinematic_frames:
			print("{}: {}".format(kinframe, kinframe.post_force_sources))

	def find_pre_kinematic_frames(self):
		for iner in self.inertial_objects:
			u = iner.unit
			iner.pre_kinematic_frames = []
			while u is not None:
				if isinstance(u, kinematic_frame):
					iner.pre_kinematic_frames.append(u)
				u = u.parent

		for kinframe in self.kinematic_frames:
			u = kinframe.parent
			kinframe.pre_kinematic_frames = []
			while u is not None:
				if isinstance(u, kinematic_frame):
					kinframe.pre_kinematic_frames.append(u)
				u = u.parent

	def find_post_force_sources(self):
		for kinframe in self.kinematic_frames:
			kinframe.post_force_sources = []

			def add_forces_recurse(u):
				if hasattr(u, "force_sources"):
					kinframe.post_force_sources.extend(u.force_sources)

				for c in u.childs:
					add_forces_recurse(c)

			add_forces_recurse(kinframe)

	def find_post_inertial_objects(self):
		for kinframe in self.kinematic_frames:
			kinframe.post_inertial_objects = []

			def add_iobj_recurse(u):
				if hasattr(u, "inertial_object"):
					kinframe.post_inertial_objects.append(u.inertial_object)

				for c in u.childs:
					add_iobj_recurse(c)

			add_iobj_recurse(kinframe)

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
		if hasattr(unit, "constrait_conditions"):
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

	def set_linear_scale(self, scale):
		self.linear_scale = scale

	def calculate_impulses(self):
		for iner in self.inertial_objects:
			accum = screw()
			for p in iner.pre_kinematic_frames:
				arm = (iner.global_pose.translation() - p.global_location.translation()) * self.linear_scale
				accum += p.global_spdscr.angular_carry(arm)
			iner.global_speed = accum

			accum = screw()
			for p in iner.pre_kinematic_frames:
				arm = (iner.global_pose.translation() - p.global_location.translation()) * self.linear_scale
				accum += p.global_accscr.angular_carry(arm)
			iner.global_acceleration = accum

			iner.update_global_impulse_with_global_speed(accum)	

	def print_impulses(self):
		for i in self.inertial_objects:
			print("{}: {}".format(i.unit, i.global_impulse))

	def onestep(self, delta):
		#for kinframe in self.kinematic_frames:
		#	kinframe.integrate_position(delta)

		self.baseunit.location_update(deep=True, view=False)
		#self.baseunit.frame_speed_update()

		for iner in self.inertial_objects:
			iner.update_globals()

		self.calculate_impulses()
		self.calculate_kinframe_frame_speeds_accelerations()
		self.calculate_kinframe_forces()
		self.calculate_kinframe_complex_inertia()
		self.calculate_kinframe_accelerations_no_constrait()

		for kinframe in self.kinematic_frames:
			kinframe.dynstep(delta)
		#	kinframe.update_local_speed()
		#	kinframe.integrate_speed(delta)
		self.baseunit.location_update(deep=True, view=False)

	def calculate_kinframe_frame_speeds_accelerations(self):
		for kinframe in self.kinematic_frames:
			kinframe.global_frame_speed = screw()
			kinframe.global_frame_acceleration = screw()
			for u in kinframe.pre_kinematic_frames:
				arm = kinframe.global_location.translation() - u.global_location.translation()
				kinframe.global_frame_speed += u.global_spdscr.kinematic_carry(arm * self.linear_scale) 
				kinframe.global_frame_acceleration += u.global_accscr.kinematic_carry(arm * self.linear_scale)



	def onestep_primitive(self, delta):
		for kinframe in self.kinematic_frames:
			kinframe.integrate_position(delta)

		self.baseunit.location_update(deep=True)

		self.calculate_kinframe_forces()
		self.calculate_kinframe_accelerations_primitive()

		for kinframe in self.kinematic_frames:
			kinframe.update_local_speed()
			kinframe.integrate_speed(delta)

	def calculate_kinframe_accelerations_no_constrait(self):
		for kinframe in self.kinematic_frames:
			kinframe.evaluate_accelerations_without_constraits()


	def calculate_kinframe_accelerations_primitive(self):
		for kinframe in self.kinematic_frames:

			if len(kinframe.post_inertial_objects) > 0:
				kinframe.global_accscr = \
					kinframe.global_force_reduction * kinframe.post_inertial_objects[0].mass

	def calculate_kinframe_complex_inertia(self):
		for kinframe in self.kinematic_frames:
			kinframe.inertia_koefficient = 0

			for iner in kinframe.post_inertial_objects:
				arm = (kinframe.global_location.translation() - iner.global_pose.translation()) * self.linear_scale
				inertia = iner.global_inertia().guigens_transform(arm)

				kinframe.inertia_koefficient += inertia.koefficient_for(kinframe.global_sensivity())

	def calculate_kinframe_forces(self):
		for kinframe in self.kinematic_frames:
			accum = screw()

			for fs in kinframe.post_force_sources:
				arm = (kinframe.global_location.translation() - fs.point()) * self.linear_scale
				f = fs.global_force().force_carry(arm)
				accum += f

			a = accum.copy()

			koriolis_enabled=True
			if koriolis_enabled:
				for iner in kinframe.post_inertial_objects:

					ispd = iner.global_speed - kinframe.global_frame_speed
					iacc = iner.global_acceleration - kinframe.global_frame_acceleration
					espd = kinframe.global_frame_speed
					eacc = kinframe.global_frame_acceleration

					arm = iner.global_pose.translation() - kinframe.global_location.translation()
					I_trans = - iner.mass * ( eacc.lin + eacc.ang.cross(arm) + espd.ang.cross(espd.ang.cross(arm)) )
					I_kor = - iner.mass * (2 * espd.ang.cross(ispd.lin))

					accum += screw(lin=I_trans + I_kor).force_carry(-arm)


			kinframe.global_force_reduction = accum
			#print("summary", kinframe.global_force_reduction)

		#print("HEERE", kinframe.global_force)

def attach_inertia(unit, 
		pose=zencad.transform.nulltrans(), 
		mass=1, 
		Ix=1, Iy=1, Iz=1, Ixy=0, Ixz=0, Iyz=0):
	unit.inertial_object = zencad.libs.inertia.inertial_object( 
		unit=unit, pose=pose, mass=1, Ix=1, Iy=1, Iz=1, Ixy=0, Ixz=0, Iyz=0)