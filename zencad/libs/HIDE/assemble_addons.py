from zencad.libs.screw import screw
from zencad.mbody.kinematic import kinematic_frame
import zencad.libs.screw
import pyservoce

def attach_speed_model(baseunit):
	def func(unit):
		unit.global_frame_speed_reference = screw()
		unit.global_frame_speed = screw()

		unit.global_frame_acceleration_reference = screw()
		unit.global_frame_acceleration = screw()


		for u in unit.childs:
			func(u)

	func(baseunit)

def update_speed_model(baseunit):
	def func(unit, prev):
		if isinstance(unit.parent, kinematic_frame):
			unit.global_frame_speed = unit.parent.global_spdscr
			#print(unit.parent.global_spdscr)

		if prev:
			reference = prev.global_frame_speed_reference
			radius = (prev.global_pose.inverse() * unit.global_pose).translation()
			arm = prev.global_pose(radius)
		else:
			reference = screw()
			arm = pyservoce.vector3()

		unit.global_frame_speed_reference = (
			reference.kinematic_carry(arm) + unit.global_frame_speed
		)

		for u in unit.childs:
			func(u, unit)

	func(baseunit, None)

	def accfunc(unit, prev):
		if isinstance(unit.parent, kinematic_frame):
			unit.global_frame_acceleration = unit.parent.global_accscr

		if prev:
			reference = prev.global_frame_acceleration_reference
			spdref = prev.global_frame_speed_reference
			radius = (prev.global_pose.inverse() * unit.global_pose).translation()
			arm = prev.global_pose(radius)
		else:
			reference = screw()
			spdref = screw()
			arm = pyservoce.vector3()

		unit.global_frame_acceleration_reference = (
			zencad.libs.screw.second_kinematic_carry(reference, spdref, arm) + unit.global_frame_acceleration
		)

		for u in unit.childs:
			accfunc(u, unit)

	accfunc(baseunit, None)
