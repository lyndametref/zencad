#!/usr/bin/env pyython3

import pyservoce
import zencad.mbody.kinematic
import zencad.libs.inertia
from zencad.libs.screw import screw

from zencad.mbody.kinematic import kinematic_frame
from zencad.libs.dynsolver import dynamic_solver

import zencad.libs.rigid_body as rigid_body
import zencad.libs.constraits as constraits
import zencad.libs.matrix_solver as matrix_solver
import zencad.libs.assemble_addons as assemble_addons

class tree_dynamic_solver(dynamic_solver):
	def __init__(self, baseunit):		
		super().__init__(baseunit)
		#self.find_post_force_sources()
		#self.find_post_inertial_objects()

		assemble_addons.attach_speed_model(baseunit)

		self.attach_finite_elements_model()
		self.reaction_solver = matrix_solver.reaction_solver(
			rigid_bodies = self.rigid_bodies, 
			constraits = self.constraits)

		#print(self.reaction_solver.rigid_bodies)

	def collect_and_atach_rigid_bodies(self):
		arr = []
		for kinframe in self.kinematic_frames:
			kinframe.rigid_body = rigid_body.rigid_body_from_kinframe(kinframe)
			#print(kinframe.name, kinframe.rigid_body)
			arr.append(kinframe.rigid_body)
		return arr

	def collect_and_atach_constraits(self):
		arr = []
		for kinframe in self.kinematic_frames:
			kinframe.constrait = constraits.make_constraits_from_kinframe(kinframe)
			arr.append(kinframe.constrait)
		return arr

	def attach_finite_elements_model(self):
		self.rigid_bodies = self.collect_and_atach_rigid_bodies()
		self.constraits = self.collect_and_atach_constraits()

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

	#def find_pre_kinematic_frames(self):
	#	for iner in self.inertial_objects:
	#		u = iner.unit
	#		iner.pre_kinematic_frames = []
	#		while u is not None:
	#			if isinstance(u, kinematic_frame):
	#				iner.pre_kinematic_frames.append(u)
	#			u = u.parent
#
	#	for kinframe in self.kinematic_frames:
	#		u = kinframe.parent
	#		kinframe.pre_kinematic_frames = []
	#		while u is not None:
	#			if isinstance(u, kinematic_frame):
	#				kinframe.pre_kinematic_frames.append(u)
	#			u = u.parent
#
	#def find_post_force_sources(self):
	#	for kinframe in self.kinematic_frames:
	#		kinframe.post_force_sources = []
#
	#		def add_forces_recurse(u):
	#			if hasattr(u, "force_sources"):
	#				#kinframe.post_force_sources.extend(u.force_sources)
	#				for fs in u.force_sources:
	#					if fs.tp == "compenser": continue
	#					if fs.tp == "node" and fs.unit is not kinframe: continue
	#					kinframe.post_force_sources.append(fs)
#
	#			for c in u.childs:
	#				add_forces_recurse(c)
#
	#		add_forces_recurse(kinframe)
#
	#def find_post_inertial_objects(self):
	#	for kinframe in self.kinematic_frames:
	#		kinframe.post_inertial_objects = []
#
	#		def add_iobj_recurse(u):
	#			if hasattr(u, "inertial_object"):
	#				kinframe.post_inertial_objects.append(u.inertial_object)
#
	#			for c in u.childs:
	#				add_iobj_recurse(c)
#
	#		add_iobj_recurse(kinframe)
#
	#def find_post_inertia_objects(self):
	#	for f in self.kinematic_frames:
	#		f.post_inertia_objects = []
#
	#		def iteration(unit):
	#			if hasattr(unit, "inertial_object"):
	#				f.post_inertia_objects.append(unit.inertial_object)
	#			for u in unit.childs:
	#				iteration(u)
#
	#		iteration(f)

	def calculate_impulses(self):
		pass
	#	for iner in self.inertial_objects:
	#		accum = screw()
	#		for p in iner.pre_kinematic_frames:
	#			arm = (iner.global_pose.translation() - p.global_pose.translation())
	#			accum += p.global_spdscr.angular_carry(arm)
	#		iner.global_speed = accum
#
	#		accum = screw()
	#		for p in iner.pre_kinematic_frames:
	#			arm = (iner.global_pose.translation() - p.global_pose.translation())
	#			accum += zencad.libs.screw.second_kinematic_carry(iacc=p.global_accscr, ispd=p.global_spdscr, arm=arm)
	#		iner.global_acceleration = accum
#
	#		iner.update_global_impulse_with_global_speed(accum)	

	def print_impulses(self):
		for i in self.inertial_objects:
			print("{}: {}".format(i.unit, i.global_impulse))

	def onestep(self, delta):
		self.baseunit.location_update(deep=True, view=False)
		
		for k in self.kinematic_frames:
			k.update_global_speed()
			k.update_global_acceleration()
			
		assemble_addons.update_speed_model(self.baseunit)

		for iner in self.inertial_objects:
			iner.update_globals()

		self.reaction_solver.update_globals()
		
		reactions = self.reaction_solver.solve()

		self.reaction_solver.apply_reactions_for_constraits(reactions)
		self.reduce_kinframe_forces()
		
		self.set_kynframe_accelerations_by_reduced_forces()

		for kinframe in self.kinematic_frames:
			kinframe.dynstep(delta)

		#self.naive_integrate()
		
		self.baseunit.location_update(deep=True, view=False)

	#def naive_integrate(self, delta):
		#for kinframe in self.kinematic_frames:
		#	inertia = kinframe.rigid_body.global_inertia
		#	kinframe.global_accscr = inertia.force_to_acceleration()
		#	kinframe.global_spdscr = kinframe.global_spdscr + kinframe.global_accscr * delta
			

	def set_kynframe_accelerations_by_reduced_forces(self):
		for kinframe in self.kinematic_frames:
			#if kinframe.parent:
			reference = kinframe.global_frame_acceleration_reference
			spdref = kinframe.global_frame_speed_reference
			radius = (kinframe.global_pose.inverse() * kinframe.output.global_pose).translation()
			arm = kinframe.global_pose(radius)
			#else:
			#	reference = screw()
			#	spdref = screw()
			#	arm = pyservoce.vector3()

			addacc = zencad.libs.screw.second_kinematic_carry(reference, spdref, arm)
			masskoeff = kinframe.rigid_body.global_inertia.koefficient_for_with_guigens(kinframe.global_sensivity())
			rforce = kinframe.force_reduction.dot(kinframe.global_sensivity())
			accel = rforce / masskoeff - addacc.dot(kinframe.global_sensivity())
			#kinframe.set_acceleration(accel)

			#print(kinframe.name, kinframe.force_reduction)
			#print(masskoeff)
			#print(accel)

		#def func(unit, lastkinframe=None):
		#	if isinstance(unit, kinematic_frame):
		#		if lastkinframe:
		#			reference = lastkinframe.global_frame_acceleration_reference
		#			spdref = lastkinframe.global_frame_speed_reference
		#			radius = (lastkinframe.global_pose.inverse() * unit.global_pose).translation()
		#			arm = lastkinframe.global_pose(radius)
		#		else:
		#			reference = screw()
		#			spdref = screw()
		#			arm = pyservoce.vector3()
#
#
		#		addacc = zencad.libs.screw.second_kinematic_carry(reference, spdref, arm)
		#		masskoeff = unit.rigid_body.global_inertia.koefficient_for_with_guigens(unit.global_sensivity())
		#		rforce = unit.force_reduction.dot(unit.global_sensivity())
		#		accel = rforce / masskoeff #+ addacc.dot(unit.global_sensivity())
#
		#		print(addacc)
#
		#		unit.set_acceleration(accel)
#
		#		lastkinframe = unit
		#
		#	for u in unit.childs:
		#		func(u, lastkinframe)
#
		#func(self.baseunit)


	def reduce_kinframe_forces(self):
		#print("reduce_kinframe_forces")
		for kinframe in self.kinematic_frames:
			accum = screw()
			accum += kinframe.rigid_body.inertia_force_in_body_frame()

			for c in kinframe.rigid_body.constrait_connections:
				reactions = c.get_reaction_force_global()

				#print(reactions)
				for r in reactions:
					accum += r

			kinframe.force_reduction = accum
			#print(accum)

	def calculate_kinframe_frame_speeds_accelerations(self):
		for kinframe in self.kinematic_frames:
			kinframe.global_frame_speed = screw()
			kinframe.global_frame_acceleration = screw()
			for u in kinframe.pre_kinematic_frames:
				arm = kinframe.global_pose.translation() - u.global_pose.translation()
				kinframe.global_frame_speed += u.global_spdscr.kinematic_carry(arm) 
				kinframe.global_frame_acceleration += \
					zencad.libs.screw.second_kinematic_carry(iacc=u.global_accscr, ispd=u.global_spdscr, arm=arm)

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
				arm = (kinframe.global_pose.translation() - iner.global_pose.translation())
				inertia = iner.global_inertia().guigens_transform(arm)

				kinframe.inertia_koefficient += inertia.koefficient_for(kinframe.global_sensivity())

	def calculate_kinframe_forces(self):
		for kinframe in self.kinematic_frames:
			accum = screw()

			for fs in kinframe.post_force_sources:
				arm = (kinframe.global_pose.translation() - fs.point())
				f = fs.global_force().force_carry(arm)
				accum += f

			a = accum.copy()

			for iner in kinframe.post_inertial_objects:

				ispd = iner.global_speed - kinframe.global_frame_speed
				iacc = iner.global_acceleration - kinframe.global_frame_acceleration
				espd = kinframe.global_frame_speed
				eacc = kinframe.global_frame_acceleration

				arm = iner.global_pose.translation() - kinframe.global_pose.translation()
				
				#I_trans = - iner.mass * ( eacc.lin + eacc.ang.cross(arm) + espd.ang.cross(espd.ang.cross(arm)) )
				#I_trans = - iner.mass * ( eacc.ang.cross(arm) + espd.ang.cross(espd.ang.cross(arm)) )
				#I_kor = - iner.mass * (2 * espd.ang.cross(ispd.lin))

				#print(iacc)

				I_trans = pyservoce.vector3()
				I_kor = pyservoce.vector3()

				#Dalamber = - iner.mass * iacc.lin
				#print(kinframe.name,iner.unit.name, screw(lin=Dalamber).force_carry(-arm))
				Dalamber = pyservoce.vector3()

				accum += screw(lin=I_trans + I_kor + Dalamber).force_carry(-arm)


			kinframe.global_force_reduction = accum
			#print("summary", kinframe.global_force_reduction)

		#print("HEERE", kinframe.global_force)


	def calculate_kinframe_free_forces(self):
		for kinframe in self.kinematic_frames:
			accum = screw()
			for fs in kinframe.local_force_sources:
				pass
				#arm = (kinframe.global_pose.translation() - fs.point())
				#f = fs.global_force().force_carry(arm)
				#print(kinframe.name, fs.unit.name, fs.__class__, f)
				#accum += f

	def calculate_kinframe_inertia_forces(self):
		for kinframe in self.kinematic_frames:
			inertia = kinframe.local_inertia
			angspd = kinframe.frame_spdscr
			kinframe.inertia_forces = screw(
				lin = - m * (angspd.cross(angspd.cross(inertia.cm))),
				ang = - angspd.cross(inertia.matrix * angspd)
			)
