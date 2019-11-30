import zencad.libs.screw
from zencad.libs.screw import screw
from zencad.transform import *
import zencad.assemble
import zencad.draw

def attach_force_model(unit, funit):
	if not hasattr(unit, "force_units"):
		unit.force_units = []

	unit.force_units.append(funit)

def evaluate_unit_force(unit):
	res = screw()
	for f in unit.force_units:
		res += f.global_force()
	return res

class force_unit:
	def __init__(self, unit, pose):
		attach_force_model(unit, self)
		self.unit = unit
		self.pose = pose

	def current_global_pose(self):
		return self.unit.global_location * self.pose

	def force(self):
		raise NotImplementedError()

	def draw_vector(self, scale=10):
		self.force()

		self.view_scale_lin = scale
		arrlen=1
		width=1
		
		self.force_lin_view = zencad.draw.arrow(
			pnt=(0,0,0), 
			vec=self._global_force.lin * self.view_scale_lin, 
			clr=zencad.color.white, 
			arrlen=arrlen, 
			width=width)

		self.unit.add_object(self.force_lin_view)

class constant_gravity_funit(force_unit):
	def __init__(self, unit, pose=nulltrans(), gravity=(0,0,-1)):
		super().__init__(unit, pose)
		self._force = screw(lin=gravity, ang=(0,0,0))

	def force(self):
		curpose = self.unit.global_location * self.pose
		self._global_force = self._force.inverse_rotate_by(curpose)
		return self._global_force

