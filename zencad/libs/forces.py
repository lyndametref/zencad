from zencad.libs.screw import screw
import pyservoce

class force_source:
	def __init__(self, finit=screw()):
		self.force = finit

	def evaluate(self):
		raise NotImplementedError

	def attach(self, unit):
		if not hasattr(unit, "force_sources"):
			unit.force_sources = []
		unit.force_sources.append(self)

class gravity(force_source):
	def __init__(self, unit, vec=pyservoce.vector3(0,0,-9.81)):
		self.g = screw(lin=vec, ang=(0,0,0))
		self.unit=unit
		self.attach(unit)

	def evaluate(self):
		pose = self.unit.global_location
		rot = pose.rotation().inverse()
		self.force = self.g.rotate_by_quat(rot) * self.unit.inertia.mass
		self.center = pyservoce.vector3(*self.unit.inertia.cm)

	def global_force(self):
		return self.g

	def point(self):
		pose = self.unit.global_location * self.unit.inertial_object.pose
		return pose.translation()

class dempher_compensate(force_source):
	def __init__(self, donor, uparent, koeff):
		self.unit=uparent
		self.donor = donor
		self.koeff = koeff
		self.attach(uparent)

	def global_force(self):
		return self.donor.unit.global_spdscr * self.koeff

	def point(self):
		pose = self.donor.unit.global_location
		return pose.translation()

class dempher(force_source):
	def __init__(self, unit, koeff):
		self.unit=unit
		self.koeff = koeff
		self.attach(unit)
		if unit.parent:
			self.mirror = dempher_compensate(donor=self, uparent=unit.parent, koeff=koeff)

	def global_force(self):
		return -self.unit.global_spdscr * self.koeff

	def point(self):
		pose = self.unit.global_location
		return pose.translation()


class motor_compensate(force_source):
	def __init__(self, donor, uparent):
		self.unit=uparent
		self.donor = donor
		self.attach(uparent)

	def global_force(self):
		return -self.donor.global_force()

	def point(self):
		return self.donor.point()

class motor(force_source):
	def __init__(self, unit):
		self.unit=unit
		self.moment=0
		self.attach(unit)
		if unit.parent:
			self.mirror = motor_compensate(donor=self, uparent=unit.parent)

	def set_moment(self, moment):
		self.moment = moment

	def global_force(self):
		return self.unit.global_sensivity() * self.moment

	def point(self):
		pose = self.unit.global_location
		return pose.translation()
