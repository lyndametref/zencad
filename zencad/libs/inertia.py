import zencad.libs.screw
import pyservoce
from zencad.libs.screw import screw

class inertia:
	def __init__(self, mass=0, matrix=pyservoce.matrix33(1,1,1), cm=pyservoce.point3(0,0,0)):
		self.cm = zencad.point3(cm)
		self.veccm = zencad.vector3(cm[0], cm[1], cm[2])
		self.matrix = matrix
		self.invmatrix = self.matrix.inverse()
		self.mass = mass

	def transform(self, trans):
		#trans = from_pose.inverse() * to_pose
		rot = trans.rotation().to_matrix()
		invrot = trans.inverse().rotation().to_matrix()
		return inertia(
			self.mass,
			invrot * self.matrix * rot,
			trans(self.cm)
		)

	def koefficient_for(self, sens):
		return (sens.lin * self.mass).length() +  (self.matrix * sens.ang).length() 

	def guigens_transform(self, mov):
		#print(mov.outerprod(mov))
		sqr = mov.length2()
		return inertia(
			matrix = self.matrix \
				+ self.mass \
					* ( pyservoce.matrix33(sqr, sqr, sqr) - mov.outerprod(mov)),
			mass = self.mass,
			cm = self.cm + mov
		)

	def impulse_to_speed(self, impulse_screw):
		lin = impulse_screw.lin / self.mass
		ang = self.invmatrix * impulse_screw.ang
		return screw(ang=zencad.vector3(ang[0,0], ang[1,0], ang[2,0]), lin=lin)

	def force_to_acceleration(self, fscr):
		return screw(
			ang=self.invmatrix * fscr.ang, 
			lin=fscr.lin / self.mass)

	def acceleration_to_force(self, accscr):
		return screw(
			ang=self.matrix * accscr.ang, 
			lin=self.mass   * accscr.lin)


	def __str__(self):
		return "".join("(m:{},i:{},c:{})".format(
			self.mass, 
			repr(self.matrix), 
			self.cm
		).split())

def guigens_transform(matrix, mov, mass):
	sqr = mov.length2()
	return matrix \
		+ mass * ( pyservoce.matrix33(sqr, sqr, sqr) - mov.outerprod(mov) )


def complex_inertia(lst):
	cm = zencad.vector3(0,0,0)
	matrix = pyservoce.matrix33()

	mass = sum([ i.mass for i in lst])
	for I in lst:
		cm += I.mass * I.veccm
	cm = cm / mass
	
	for I in lst:
		matrix += guigens_transform(I.matrix, cm - I.veccm, I.mass)

	return inertia(
		mass, matrix, zencad.point3(*cm)
	)

class inertial_object:
	def __init__(self, unit, pose, mass, Ix, Iy, Iz, Ixy, Ixz, Iyz):
		self.unit = unit
		self.pose = pose
		self.mass = mass
		self.global_impulse = screw()
		self.matrix = pyservoce.matrix33(
			Ix, Ixy,Ixz,
			Ixy,Iy ,Iyz,
			Ixz,Iyz,Iz 
		)
		self.update_globals()

	def global_inertia(self):
		cm = pyservoce.point3(*self.global_pose.translation())
		return inertia(mass=self.mass, matrix=self.global_matrix, cm=cm)

	def transformed_matrix(self, trans):
		rot = trans.rotation().to_matrix()
		transrot = rot.transpose()
		return transrot * self.matrix * rot
		#return self.matrix

	def update_globals(self):
		self.global_pose = self.unit.global_location * self.pose
		self.global_matrix = self.transformed_matrix(self.global_pose)

	def __repr__(self):
		return "".join("(m:{},i:{},c:{})".format(
			self.mass, 
			repr(self.matrix), 
			self.pose
		).split())

	def update_global_impulse_with_global_speed(self, global_spdscr):
		self.global_impulse = screw(
			lin = self.mass * global_spdscr.lin,
			ang = self.global_matrix * global_spdscr.ang
		)