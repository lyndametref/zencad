from zencad.libs.screw import screw
from zencad.libs.kinematic import kinematic_frame

def attach_speed_model(baseunit):
	def func(unit):
		unit.global_frame_speed_reference = screw()
		unit.global_frame_speed = screw()

		for u in unit.childs:
			func(u)

	func(baseunit)

def update_speed_model(baseunit):
	#print("update_speed_model")
	def func(unit, reference):
		if isinstance(unit, kinematic_frame):
			unit.global_frame_speed = unit.global_spdscr

		#print(unit, unit.global_frame_speed, reference)
		unit.global_frame_speed_reference = reference + unit.global_frame_speed

		for u in unit.childs:
			func(u, unit.global_frame_speed_reference)

	func(baseunit, screw())