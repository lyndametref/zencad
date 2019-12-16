from zencad.mbody.kinematic import *
import zencad.mbody.kinematic as kinematic
import zencad.libs.inertia
import zencad.transform

class dynamic_solver:
	def __init__(self, baseunit):
		self.baseunit = baseunit

		baseunit.location_update(deep=True, view=False)
		
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
		self.find_pre_kinematic_frames()

		self.find_local_inertial_objects()
		self.find_local_force_sources()

		self.set_kinematic_frames_crosslinks()

		self.evaluate_kinframes_inertia()		

		baseunit.location_update(deep=True, view=False)

	def set_kinematic_frames_crosslinks(self):
		def func(unit, last_kinframe):
			unit.base_kinframe = last_kinframe
			
			if isinstance(unit, kinematic_frame):
				last_kinframe = unit

			for u in unit.childs:
				func(u, last_kinframe)

		func(self.baseunit, None)

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

	def find_all_inertial_objects(self, unit, retarr):
		if hasattr(unit, "inertial_object"):
			retarr.append(unit.inertial_object)

		for u in unit.childs:
			self.find_all_inertial_objects(u, retarr)

	def find_all_kinematic_frames(self, unit, retarr):
		if isinstance(unit, zencad.mbody.kinematic.kinematic_frame):
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

	#def for_each_units_in_kinframe(self, unit, doit):
	#	print("for_each_units_in_kinframe", unit)
	#	def func(unit):
	#		doit(unit)
	#		for u in unit.childs:
	#			if isinstance(u, kinematic_frame):
	#				continue
	#			func(u)
	#	func(unit)		

	def find_local_inertial_objects(self):
		for kinframe in self.kinematic_frames:
			kinframe.local_inertial_objects = []
			def doit(unit):
				if hasattr(unit, "inertial_object"):
					kinframe.local_inertial_objects.append(unit.inertial_object)
					unit.inertial_object.update_globals()
			kinematic.for_each_units_in_kinframe(kinframe, doit)

	def find_local_force_sources(self):
		for kinframe in self.kinematic_frames:
			kinframe.local_force_sources = []
			def doit(unit):
				if hasattr(unit, "force_sources"):
					kinframe.local_force_sources.extend(unit.force_sources)
					for fs in unit.force_sources:
						fs.kinframe_pose = kinframe.global_pose.inverse() * fs.global_pose
			kinematic.for_each_units_in_kinframe(kinframe, doit)

	def print_local_inertial_objects(self):
		for kinframe in self.kinematic_frames:
			print(kinframe.local_inertial_objects)

	def evaluate_kinframes_inertia(self):
		pass
		#for kinframe in self.kinematic_frames:
		#	for iner in kinframe.local_inertial_objects:
		#		iner.update_globals()
		#	kinematic_frame.global_frame_inertia = inertia.complex_inertia()

		#for kinframe in self.kinematic_frames:
		#	arr = []
		#	for iner in kinframe.local_inertial_objects:
		#		etrans = kinframe.global_pose * iner.global_pose.inverse() 
		#		arr.append(iner.get_transformed_inertia(etrans))
		#	#print(arr)
		#	kinframe.frame_inertia = zencad.libs.inertia.complex_inertia(arr)




									