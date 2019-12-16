#!/usr/bin/env python3

import pyservoce
from zencad.libs.screw import screw

class constrait:
	def __init__(self):
		self.connections = []

	def constrait_screws(self):
		raise NotImplementedError
		
	def attach_positive_connection(self, body, radius):
		c = constrait_connection_positive(self, body, radius)
		self.connections.append(c)
		body.constrait_connections.append(c)

	def attach_negative_connection(self, body, radius):
		c = constrait_connection_negative(self, body, radius)
		self.connections.append(c)
		body.constrait_connections.append(c)

	def update_globals(self):
		for c in self.connections:
			c.update_globals()

class rotator_constrait(constrait):
	def __init__(self, ax):
		super().__init__()
		self.axis = pyservoce.vector3(ax)
		self.__dbg = 0

	def rank(self):
		return 5

	def constrait_screws(self):
		ang = self.axis
		#print(ang)

		#self.__dbg += 1
		#if self.__dbg == 50:
		#	exit(0)

		if ang.early(pyservoce.vector3(1,0,0), 0.00001):
			r = pyservoce.vector3(0,1,0)
		else:
			r = pyservoce.vector3(1,0,0)

		f = ang.cross(r).normalize()
		s = ang.cross(f)

		scrs = [
			screw(lin=(1,0,0), ang=(0,0,0)),
			screw(lin=(0,1,0), ang=(0,0,0)),
			screw(lin=(0,0,1), ang=(0,0,0)),
			screw(lin=(0,0,0), ang=f),
			screw(lin=(0,0,0), ang=s),
		]

		#print(scrs)

		return scrs

class constrait_connection:
	def __init__(self, constrait, body, radius):
		if not isinstance(radius, pyservoce.vector3):
			raise Exception("not vector")

		self.constrait = constrait
		self.body = body
		self.radius = radius

	def get_reaction_force_global(self):
		reactions = self.constrait.reactions
		sreactions = [ (self.body_carried_constrait_screws()[i] * reactions[i][0]) for i in range(self.rank()) ]

		return sreactions

	def body_carried_constrait_screws(self):
		scrs = self.constrait_screws()
		arm = self.body.global_pose(self.radius)
		#print(self.body.global_pose)
		#print("arm", arm)


		tscrs = [ s.force_carry(-arm) for s in scrs ]
		#print(tscrs)

		return tscrs

	def update_globals(self):
		pass
		#self.global_pose = self.body.global_pose * self.pose

	def rank(self):
		return self.constrait.rank()

class constrait_connection_positive(constrait_connection):
	def __init__(self, constrait, body, radius):
		super().__init__(constrait, body, radius)	

	def constrait_screws(self):
		return self.constrait.constrait_screws()

class constrait_connection_negative(constrait_connection):
	def __init__(self, constrait, body, radius):
		super().__init__(constrait, body, radius)	

	def constrait_screws(self):
		return [ 
			-scr for scr in self.constrait.constrait_screws()
		]


#class constrait_component:
#	def __init__(self, reference)
#		self.reference_object = reference

def make_constraits_from_kinframe(kinframe):
	c = kinframe.constrait

	post = kinframe.rigid_body
	c.attach_positive_connection(body=post, radius=pyservoce.vector3(0,0,0))

	if kinframe.base_kinframe:
		pre = kinframe.base_kinframe.rigid_body
		radius =(pre.global_pose.inverse() * kinframe.global_pose).translation()
		#print(kinframe.name, radius)
		#exit()
		c.attach_negative_connection(
			body=pre, 
			radius= radius
		)


	return c
