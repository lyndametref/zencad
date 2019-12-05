import zencad.libs.screw
import pyservoce
from zencad.libs.screw import screw
import numpy

class inertia:
	def __init__(self, mass=0, matrix=numpy.diag([1,1,1]), cm=pyservoce.point3(0,0,0)):
		self.cm = zencad.point3(cm)
		self.matrix = numpy.matrix(matrix)
		self.invmatrix = numpy.linalg.inv(self.matrix)
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

	def impulse_to_speed(self, impulse_screw):
		lin = impulse_screw.lin / self.mass
		ang = self.invmatrix * numpy.asarray(impulse_screw.ang).reshape((3,1))
		return screw(ang=zencad.vector3(ang[0,0], ang[1,0], ang[2,0]), lin=lin)

	def force_to_acceleration(self, fscr):
		lin = fscr.lin / self.mass
		ang = self.invmatrix * numpy.asarray(fscr.ang).reshape((3,1))
		return screw(ang=zencad.vector3(ang[0,0], ang[1,0], ang[2,0]), lin=lin)

	def __str__(self):
		return "".join("(m:{},i:{},c:{})".format(
			self.mass, 
			repr(self.matrix), 
			self.cm
		).split())

def guigens_transform(matrix, mov, mass):
	return matrix + mass * ( ((mov.dot(mov))**2 * numpy.diag([1.,1.,1.])) - numpy.ma.outerproduct(mov, mov) )


def complex_inertia(lst):
	cm = zencad.vector3(0,0,0)
	matrix = numpy.matrix('0 0 0; 0 0 0; 0 0 0', dtype=numpy.float64)

	mass = sum([ i.mass for i in lst])
	for I in lst:
		cm += I.mass * zencad.vector3(*I.cm)
	cm = cm / mass
	
	for I in lst:
		matrix += guigens_transform(I.matrix, numpy.array([*(cm - zencad.vector3(*I.cm))]), I.mass)

	return inertia(
		mass, matrix, zencad.point3(*cm)
	)

class inertial_object:
	def __init__(self, unit, pose, mass, Ix, Iy, Iz, Ixy, Ixz, Iyz):
		self.unit = unit
		self.pose = pose
		self.mass = mass
		self.global_impulse = screw()
		self.matrix = numpy.matrix([
			[Ix, Ixy,Ixz],
			[Ixy,Iy ,Iyz],
			[Ixz,Iyz,Iz ]
		])
		self.update_globals()

	def global_inertia(self):
		cm = pyservoce.point3(*self.global_pose.translation())
		return inertia(mass=self.mass, matrix=self.global_matrix, cm=cm)

	def transformed_matrix(self, trans):
		rot = trans.rotation().to_matrix()
		invrot = trans.inverse().rotation().to_matrix()
		return invrot * self.matrix * rot
		#return self.matrix

	def update_globals(self):
		self.global_pose = self.unit.global_location * self.pose
		self.global_matrix = self.transformed_matrix(
		self.global_pose.inverse())

	def __repr__(self):
		return "".join("(m:{},i:{},c:{})".format(
			self.mass, 
			repr(self.matrix), 
			self.pose
		).split())

	def update_global_impulse_with_global_speed(self, global_spdscr):
		l = numpy.matmul(self.global_matrix, numpy.array(global_spdscr.ang))
		print(l)
		self.global_impulse = screw(
			lin = self.mass * global_spdscr.lin,
			ang = pyservoce.vector3(*numpy.asarray(l)[0])
		)