import zencad.assemble
import zencad.libs.physics
import zencad.libs.screw
import zencad.libs.inertia
from zencad.libs.screw import screw
from zencad.libs.inertia import inertia
import pyservoce
import numpy

from abc import ABC, abstractmethod

#class kynematic_unit_output(zencad.assemble.unit):
#	def __init__(self, *args, **kwargs):
#		super().__init__(*args, **kwargs)

class kinematic_frame(zencad.assemble.unit):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.global_force_reduction = screw()
		self.global_spdscr = screw()
		self.complex_inertia = inertia()

	def link(self, arg):
		self.link_unique(arg)

	def link_unique(self, arg):
		self.childs = set()
		super().link(arg)
		self.output = arg

	def evaluate_accelerations_without_constraits(self):
		raise NotImplementedError

	def update_global_speed(self):
		self.global_spdscr = self.spdscr.rotate_by(self.global_location)

	def update_local_speed(self):
		self.spdscr = self.global_spdscr.inverse_rotate_by(self.global_location)

	def set_speed_screw(self, spdscr):
		self.spdscr = spdscr
		self.update_global_speed()

	def angular_speed(self):
		return self.spdscrew.ang

	def linear_speed(self):
		return self.spdscrew.lin
	
	def reduce_forces_as_prereaction(self):
		self.output.reduce_forces()
		trans = self.output.location
		mov = - trans.translation()
		rot = trans.rotation().inverse()
		self.prereaction = self.output.reaction.rotate_by_quat(rot).carry(-mov)

	def reduce_forces(self):
		self.reduce_forces_as_prereaction()
		self.divide_prereaction()

	def divide_prereaction(self):
		raise NotImplementedError

	def integrate_position(self, delta):
		return

	def integrate_speed(self, delta):
		return

class space(kinematic_frame):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

	def integrate_position(self, delta):
		self.location = self.location * self.spdscr.to_trans()

	def integrate_speed(self, delta):
		self.global_spdscr = self.global_spdscr + self.global_accscr * delta

#class free(kinematic_frame):
#	def divide_prereaction(self):
#		self.reaction = zencad.libs.screw.screw((0,0,0),(0,0,0))
#		self.dalamber = self.prereaction

class kinematic_unit(ABC, kinematic_frame):
	"""Кинематическое звено задаётся двумя системами координат,
	входной и выходной. Изменение кинематических параметров изменяет
	положение выходной СК относительно входной"""

	def __init__(self, **kwargs):
		super().__init__(**kwargs)
		#self.output = kynematic_unit_output(parent=self)

	@abstractmethod
	def senses(self):
		"""Возвращает кортеж тензоров производной по положению
		по набору кинематических координат
		в собственной системе координат в формате (w, v)"""

		raise NotImplementedError

	@abstractmethod
	def set_coords(self, coords, **kwargs):
		"""Устанавливает модельное положение звена согласно 
		переданным координатам"""
		
		raise NotImplementedError

class kinematic_unit_one_axis(kinematic_unit):
	"""Кинематическое звено специального вида,
	взаимное положение СК которого может быть описано одним 3вектором

	ax - вектор, задающий ось и направление. задаётся с точностью до длины.
	mul - линейный коэффициент масштабирования входной координаты.
	"""
	
	def __init__(self, ax, mul=1, **kwargs):
		super().__init__(**kwargs)
		self.coord = 0
		self.ax = pyservoce.vector3(ax)
		self.ax = self.ax.normalize()
		self.mul = mul
		self.axmul = self.ax * self.mul
		self.speed = 0
		self.acceleration = 0
		self.dempher_koeff = 1

	#override
	def senses(self):
		return (self.sensivity(),)

	#override
	def set_coords(self, coords, **kwargs):
		self.set_coord(coords[0], **kwargs)

	@abstractmethod
	def sensivity(self):
		raise NotImplementedError

	def sensivity_array(self):
		s = self.sensivity()
		return numpy.array([s[0][0],s[0][1],s[0][2],s[1][0],s[1][1],s[1][2]])

	def sensivity_screw(self):
		s = self.sensivity()
		return screw(ang=s[0],lin=s[1])

	def dynstep(self, delta):
		self.coord += self.speed * delta 
		self.speed += self.acceleration * delta - self.speed * self.dempher_koeff * delta
		self.set_coord(self.coord, deep=False, view=False)

	@abstractmethod
	def set_coord(self, coord, **kwargs):
		raise NotImplementedError

	def evaluate_accelerations_without_constraits(self):
		#print("global_force_reduction", self.global_force_reduction)
		carried_force_reduction = self.global_force_reduction.carry(pyservoce.vector3(*self.complex_inertia.cm))
		#print("carried_force_reduction", carried_force_reduction)
		posible_acceleration = self.complex_inertia.force_to_acceleration(carried_force_reduction)
		#print("posible_acceleration", posible_acceleration)
		posible_acceleration = posible_acceleration.carry(-pyservoce.vector3(*self.complex_inertia.cm))
		#print("carried_posible_acceleration", posible_acceleration)
		#print(self.complex_inertia)
		#print(posible_acceleration)
		sens = self.sensivity_screw().rotate_by(self.global_location)
		#print(sens)
		self.acceleration = posible_acceleration.dot(sens)

class rotator(kinematic_unit_one_axis):
	def sensivity(self):
		"""Возвращает тензор производной по положению
		в собственной системе координат в формате (w, v)"""
		return (self.axmul, pyservoce.vector3())

	def set_coord(self, coord, **kwargs):
		self.coord = coord
		self.output.relocate(pyservoce.rotate(self.ax, coord * self.mul), **kwargs)

class actuator(kinematic_unit_one_axis):
	def sensivity(self):
		"""Возвращает тензор производной по положению
		в собственной системе координат в формате (w, v)"""
		return (pyservoce.vector3(), self.axmul)

	def set_coord(self, coord, **kwargs):
		self.coord = coord
		self.output.relocate(pyservoce.translate(self.ax * coord * self.mul), **kwargs)


class planemover(kinematic_unit):
	"""Кинематическое звено с двумя степенями свободы для перемещения
	по плоскости"""

	def __init__(self):
		super().__init__(**kwargs)
		self.x = 0
		self.y = 0

	def senses(self):
		return (
			(pyservoce.vector3(1,0,0), pyservoce.vector3()),
			(pyservoce.vector3(0,1,0), pyservoce.vector3())
		)

	def set_coords(self, coords, **kwargs):
		self.x = coord[0]
		self.y = coord[1]
		self.output.relocate(
			pyservoce.translate(pyservoce.vector3(self.x, self.y, 0)), **kwargs)


class spherical_rotator(kinematic_unit):
	def __init__(self, **kwargs):
		super().__init__(**kwargs)
		self._yaw = 0
		self._pitch = 0

	def senses(self):
		raise NotImplementedError
		#return (
		#	(pyservoce.vector3(1,0,0), pyservoce.vector3()),
		#	(pyservoce.vector3(0,1,0), pyservoce.vector3())
		#)

	def set_yaw(self, angle, **kwargs):
		self._yaw = angle
		self.update_position(**kwargs)

	def set_pitch(self, angle, **kwargs):
		self._pitch = angle
		self.update_position(**kwargs)

	def set_coords(self, coords, **kwargs):
		self._yaw = coord[0]
		self._pitch = coord[1]
		self.update_position(**kwargs)

	def update_position(self, **kwargs):
		self.output.relocate(pyservoce.rotateZ(self._yaw) * pyservoce.rotateY(self._pitch))

class kinematic_chain:
	"""Объект-алгоритм управления участком кинематической чепи от точки
	выхода, до точки входа.

	Порядок следования обратный, потому что звено может иметь одного родителя,
	но много потомков. Цепочка собирается по родителю.

	finallink - конечное звено.
	startlink - начальное звено. 
		Если не указано, алгоритм проходит до абсолютной СК"""

	def __init__(self, finallink, startlink = None):
		self.chain = self.collect_chain(finallink, startlink)
		self.simplified_chain = self.simplify_chain(self.chain)
		self.kinematic_pairs = self.collect_kinematic_pairs()

	def collect_kinematic_pairs(self):
		par = []
		for l in self.chain:
			if isinstance(l, kinematic_unit):
				par.append(l)
		return par

	#def collect_coords(self):
	#	arr = []
	#	for l in self.parametered_links:
	#		arr.append(l.coord)
	#	return arr

	@staticmethod
	def collect_chain(finallink, startlink = None):
		chain = []
		link = finallink

		while link is not startlink:
			chain.append(link)
			link = link.parent

		if startlink is not None:
			chain.append(startlink)

		return chain

	@staticmethod
	def simplify_chain(chain):
		ret = []
		tmp = None

		for l in chain:
			if isinstance(l, kinematic_unit):
				if tmp is not None:
					ret.append(tmp)
					tmp = None
				ret.append(l)
			else:
				if tmp is None:
					tmp = l.location
				else:
					tmp = tmp * l.location

		if tmp is not None:
			ret.append(tmp)

		return ret

	def getchain(self):
		return self.chain

	def sensivity(self, basis=None):
		"""Вернуть массив тензоров производных положения выходного
		звена по вектору координат в виде [(w_i, v_i) ...]"""

		trsf = pyservoce.nulltrans()
		senses = []

		outtrans = self.chain[0].global_location

		"""Два разных алгоритма получения масива тензоров чувствительности.
		Первый - проход по цепи с аккумулированием тензора трансформации.
		Второй - по глобальным объектам трансформации

		Возможно следует использовать второй и сразу же перегонять в btrsf вместо outtrans"""

		if False:
			for link in self.simplified_chain:
				if isinstance(link, pyservoce.libservoce.transformation):
					trsf = link * trsf
				
				else:
					lsenses = link.senses()
					radius = trsf.translation()
				
					for sens in reversed(lsenses):
						
						wsens = sens[0]
						vsens = wsens.cross(radius) + sens[1]
				
						itrsf = trsf.inverse()
				
						senses.append((
							itrsf(wsens), 
							itrsf(vsens)
						))
				
					trsf = link.location * trsf

		else:
			for link in self.kinematic_pairs:
				lsenses = link.senses()
				
				linktrans = link.output.global_location
				trsf = linktrans.inverse() * outtrans
			
				radius = trsf.translation()
			
				for sens in reversed(lsenses):
					
					wsens = sens[0]
					vsens = wsens.cross(radius) + sens[1]
				
					itrsf = trsf.inverse()
				
					senses.append((
						itrsf(wsens), 
						itrsf(vsens)
					))

		"""Для удобства интерпретации удобно перегнать выход в интуитивный базис."""
		if basis is not None:
			btrsf = basis.global_location
			#trsf =  btrsf * outtrans.inverse()
			#trsf =  outtrans * btrsf.inverse() #ok
			trsf =  btrsf.inverse() * outtrans #ok
			#trsf =  outtrans.inverse() * btrsf
			#trsf =  trsf.inverse()

			senses = [ (trsf(w), trsf(v)) for w, v in senses ]

		return list(reversed(senses))



def attach_kinematic_chains(unit):
	for u in unit.childs:
		attach_kinematic_chains(u)

	unit.kinematic_chain = kinematic_chain(unit)
