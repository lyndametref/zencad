#!/usr/bin/env python3

import zencad.libs.inertia
import zencad.mbody.kinematic as kinematic
from zencad.libs.screw import screw


class rigid_body:
	def __init__(self, inertia, global_pose, mirror=None):
		self.inertia = inertia
		self.global_pose = global_pose
		self.global_speed = screw() 
		self.fsources = []
		self.constrait_connections = []
		self.mirror = mirror
		self.update_globals()

	def set_global_pose(self, global_pose):
		self.global_pose = global_pose

	def set_global_speed(self, global_speed):
		self.global_speed = global_speed

	def update_globals(self):
		if self.mirror:
			self.global_pose = self.mirror.global_pose
			self.global_speed = self.mirror.global_frame_speed_reference

		self.global_inertia = self.inertia.rotate(self.global_pose)	
		#print(self.global_inertia.radius)
		self.global_mass_matrix = self.global_inertia.to_mass_matrix()	
		#print(self.global_pose)

	def inertia_force(self):
		aspd = self.global_speed.ang
		mass = self.global_inertia.mass
		rho = self.global_inertia.radius
		imat = self.global_inertia.matrix

		ret = screw(
			lin= - mass * aspd.cross(aspd.cross(rho)),
			ang= - aspd.cross(imat*aspd)
		)

		#if self.mirror.parent.name == "BROT":
		#	print(ret)
		
		return ret

	def inertia_force_in_body_frame(self):
		arm = -self.global_inertia.radius
		return self.inertia_force().force_carry(arm)

def kinframe_inertial_objects(kinframe):
	inertial_objects = []
	def doit(unit):
		if hasattr(unit, "inertial_object"):
			inertial_objects.append(unit.inertial_object)
			unit.inertial_object.update_globals()
	kinematic.for_each_units_in_kinframe(kinframe, doit)

	#print(inertial_objects)
	return inertial_objects


def kinframe_force_sources(kinframe):
	#TRANSLATION?
	force_sources = []
	def doit(unit):
		if hasattr(unit, "force_sources"):
			force_sources.extend(unit.force_sources)
			for fs in unit.force_sources:
				fs.update_globals()
				fs.kinframe_pose = kinframe.global_pose.inverse() * fs.global_pose
	kinematic.for_each_units_in_kinframe(kinframe, doit)
	return force_sources


#def kinframe_constraits(kinframe):
#	constraits = []
#	def doit(unit):
#		if hasattr(unit, "constraits"):
#			constraits.extend(unit.constraits)
#			for c in unit.constraits:
#				c.update_globals()
#				c.kinframe_pose = kinframe.global_pose.inverse() * c.global_pose
#	kinematic.for_each_units_in_kinframe(kinframe, doit)
#	return constraits

def inertia_of_objects(kinframe, iobjects):
	arr = []
	for iner in iobjects:
		mov = (kinframe.output.global_pose.inverse() * iner.unit.global_pose).translation()
		mov = kinframe.output.global_pose(mov)

		#print(kinframe.global_pose.inverse() * iner.unit.global_pose)
		#print(kinframe.global_pose * iner.unit.global_pose.inverse())
		#print(iner.unit.global_pose.inverse() * kinframe.global_pose)
		#print(iner.unit.global_pose * kinframe.global_pose.inverse())

		#print(mov)
		inertia = iner.get_rotated_inertia(iner.unit.global_pose)
		inertia.radius += mov
		arr.append(inertia)
		#exit(0)

	result = zencad.libs.inertia.complex_inertia(arr)
	result = result.rotate(kinframe.output.global_pose.inverse())

	#print(result)
	#exit(0)

	return result


def rigid_body_from_kinframe(kinframe):
	inertial_objects = kinframe_inertial_objects(kinframe)
	inertia = inertia_of_objects(kinframe, inertial_objects)
	#fsources = kinframe_force_sources(kinframe)

	#print(inertia)
	return rigid_body(
		inertia = inertia, 
		global_pose = kinframe.output.global_pose,
		mirror=kinframe.output
	)
	