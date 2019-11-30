import zencad.libs.screw
import zencad
from zencad.libs.screw import screw
import numpy

class inertia:
	def __init__(self, mass, matrix, cm):
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

	def __str__(self):
		return "".join("(m:{},i:{},c:{})".format(
			self.mass, 
			repr(self.matrix), 
			self.cm
		).split())

def guigens_transform(matrix, mov, mass):
	return matrix + mass * ( (mov.length()**2 * numpy.diag([1.,1.,1.])) - mov.outerprod(mov) )


def complex_inertia(lst):
	cm = zencad.vector3(0,0,0)
	matrix = numpy.matrix('0 0 0; 0 0 0; 0 0 0', dtype=numpy.float64)

	mass = sum([ i.mass for i in lst])
	for I in lst:
		cm += I.mass * zencad.vector3(*I.cm)
	cm = cm / mass
	
	for I in lst:
		matrix += guigens_transform(I.matrix, cm - zencad.vector3(*I.cm), I.mass)
	
	return inertia(
		mass, matrix, zencad.point3(*cm)
	)
